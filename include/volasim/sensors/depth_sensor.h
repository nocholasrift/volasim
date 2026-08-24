#ifndef DEPTHSENSOR_H
#define DEPTHSENSOR_H

#include <volasim/comms/msgs/DepthCamera.pb.h>
#include <volasim/sensors/sensor_handoff.h>
#include <volasim/simulation/dynamic_object.h>
#include <volasim/simulation/entity.h>
#include <volasim/simulation/gl_resource.h>
#include <volasim/simulation/rate_gate.h>
#include <volasim/simulation/shader.h>
#include <volasim/simulation/world_buffer.h>

#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <pugixml.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Renders the scene from the sensor's viewpoint into a depth-only FBO.
static const std::string depth_vertex_shader =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "  gl_Position = mvp * vec4(aPos, 1.0);\n"
    "}\n";

static const std::string depth_fragment_shader =
    "#version 330 core\n"
    "void main() {}\n";

static const std::string point_vertex_shader =
    "#version 330 core\n"
    "uniform sampler2D depth_tex;\n"
    "uniform mat4 sensor_inv_vp;\n"
    "uniform mat4 main_mvp;\n"
    "uniform vec2 sensor_size;\n"
    "void main() {\n"
    "  int j = gl_VertexID % int(sensor_size.x);\n"
    "  int row = gl_VertexID / int(sensor_size.x);\n"  // row 0 = bottom (OpenGL convention)
    "  float px = float(j) + 0.5;\n"  // sample/reconstruct at pixel center
    "  float py = float(row) + 0.5;\n"
    "  vec2 uv = vec2(px / sensor_size.x, py / sensor_size.y);\n"
    "  float d = texture(depth_tex, uv).r;\n"
    "  if (d >= 1.0) {\n"
    "    gl_Position = vec4(0.0, 0.0, 2.0, 1.0);\n"  // behind far plane — clipped
    "    gl_PointSize = 0.0;\n"
    "    return;\n"
    "  }\n"
    "  float x_ndc = 2.0 * px / sensor_size.x - 1.0;\n"
    "  float y_ndc = 2.0 * py / sensor_size.y - 1.0;\n"
    "  float z_ndc = 2.0 * d - 1.0;\n"
    "  vec4 world_pos = sensor_inv_vp * vec4(x_ndc, y_ndc, z_ndc, 1.0);\n"
    "  world_pos /= world_pos.w;\n"
    "  gl_Position = main_mvp * world_pos;\n"
    "  gl_PointSize = 1.0;\n"
    "}\n";

static const std::string point_fragment_shader =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    "  FragColor = vec4(0.0, 0.0, 1.0, 1.0);\n"
    "}\n";

// Depth buffer -> uint16 mm, so the readback ships compact metric depth like a
// real RGBD sensor instead of raw float window depth.
static const std::string depth_convert_vertex_shader =
    "#version 330 core\n"
    "out vec2 v_uv;\n"
    "void main() {\n"
    "  vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));\n"
    "  v_uv = p;\n"
    "  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);\n"
    "}\n";

static const std::string depth_convert_fragment_shader =
    "#version 330 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D depth_tex;\n"
    "uniform vec2 z_range;\n"  // x = near, y = far
    "out uint depth_mm;\n"
    "void main() {\n"
    "  float d = texture(depth_tex, v_uv).r;\n"
    "  if (d >= 1.0) {\n"
    "    depth_mm = 0u;\n"  // background / no return -> 0 (invalid)
    "    return;\n"
    "  }\n"
    "  float z_ndc = 2.0 * d - 1.0;\n"
    "  float zn = z_range.x;\n"
    "  float zf = z_range.y;\n"
    "  float depth_m = (2.0 * zn * zf) / (zf + zn - z_ndc * (zf - zn));\n"
    "  depth_mm = uint(clamp(depth_m * 1000.0, 0.0, 65535.0));\n"
    "}\n";

class GPUSensor {
 public:
  enum class Type { kLidar = 0, kDepthCamera };

  struct Settings {
    float width;
    float height;
    float fx, fy;
    float cx, cy;
    float z_near, z_far;
    float rate_hz;  // capture/broadcast rate for this sensor, from its XML

    GPUSensor::Type type;
  };

  GPUSensor(const Settings& settings, Entity* parent)
      : settings_(settings), parent_(parent) {

    if (settings_.width == 0 || settings_.height == 0)
      throw std::invalid_argument("[GPUSensor] width/height must be > 0");
    if (settings_.fx <= 0 || settings_.fy <= 0)
      throw std::invalid_argument("[Settings] fx/fy must be > 0");
    if (settings_.z_near <= 0 || settings_.z_far <= settings_.z_near)
      throw std::invalid_argument("[GPUSensor] z_near > 0 and z_far > z_near");
    if (!std::isfinite(settings_.rate_hz) || settings_.rate_hz <= 0.F)
      throw std::invalid_argument("[GPUSensor] rate_hz must be finite and > 0");

    num_points_ = settings_.width * settings_.height;
    setProjectionMatrix();
  }

  static GPUSensor fromXML(const pugi::xml_node& root, Entity* parent) {
    auto get_value = [&root](const std::string& key) {
      return std::stof(root.child_value(key.c_str()));
    };

    auto get_value_or = [&root](const std::string& key, float fallback) {
      const std::string value = root.child_value(key.c_str());
      return value.empty() ? fallback : std::stof(value);
    };

    float hfov_rad = (M_PI / 180.F) * get_value("hfov_deg");
    float vfov_rad = (M_PI / 180.F) * get_value("vfov_deg");

    Settings settings;
    settings.width   = get_value("width");
    settings.height  = get_value("height");
    settings.fx      = (settings.width / 2.F) / tan(hfov_rad / 2.F);
    settings.fy      = (settings.height / 2.F) / tan(vfov_rad / 2.F);
    settings.cx      = settings.width / 2.F;
    settings.cy      = settings.height / 2.F;
    settings.z_near  = get_value("z_near");
    settings.z_far   = get_value("z_far");
    settings.rate_hz = get_value_or("rate_hz", 30.F);

    // TODO: just use depth camera for now, eventually implement lidar
    settings.type = GPUSensor::Type::kDepthCamera;

    return GPUSensor(settings, parent);
  }

  void init() {
    glGenFramebuffers(1, fbo_.addr());
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_.get());

    glGenTextures(1, depth_tex_.addr());
    glBindTexture(GL_TEXTURE_2D, depth_tex_.get());
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, settings_.width,
                 settings_.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           depth_tex_.get(), 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      throw std::runtime_error(
          "[GPU Sensor] Framebuffer initialization failed!");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // empty VAO — point shader uses gl_VertexID, no vertex attributes needed
    glGenVertexArrays(1, vao_.addr());

    point_shader_ =
        std::make_unique<Shader>(point_vertex_shader, point_fragment_shader);
    depth_shader_ =
        std::make_unique<Shader>(depth_vertex_shader, depth_fragment_shader);

    initReadback();
  }

  // Renders the scene into the sensor's depth FBO. Call once per frame before draw().
  void update(const Entity& world, const WorldSnapshot& snapshot,
              Shader& shader) {
    sensor_view_mat_ = getViewMat(snapshot);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_.get());
    glViewport(0, 0, settings_.width, settings_.height);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);

    glUseProgram(depth_shader_->getID());
    world.draw(snapshot, glm::mat4(1.F), sensor_view_mat_, proj_mat_,
               *depth_shader_);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  }

  // Draws the point cloud using the main camera's view/proj matrices.
  void draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
            Shader& shader) {
    glUseProgram(point_shader_->getID());

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depth_tex_.get());
    point_shader_->setUniformInt("depth_tex", 0);
    point_shader_->setUniformMat4("sensor_inv_vp",
                                  glm::inverse(proj_mat_ * sensor_view_mat_));
    point_shader_->setUniformMat4("main_mvp", proj_mat * view_mat);
    point_shader_->setUniformVec2("sensor_size",
                                  glm::vec2(settings_.width, settings_.height));

    glDepthFunc(GL_LEQUAL);
    glEnable(GL_POLYGON_OFFSET_POINT);
    glPolygonOffset(-1.0F, -1.0F);
    glBindVertexArray(vao_.get());
    glDrawArrays(GL_POINTS, 0, num_points_);
    glBindVertexArray(0);
    glDisable(GL_POLYGON_OFFSET_POINT);
    glDepthFunc(GL_LESS);

    glBindTexture(GL_TEXTURE_2D, 0);
  }

  glm::mat4 getViewMat(const WorldSnapshot& snapshot) const {
    // lookAt needs a position and two basis vectors, which are just columns of
    // the global transform (glm is column-major) — no decomposition needed.
    glm::mat4 tf      = parent_->getGlobalTransform(snapshot);
    auto      pos     = glm::vec3(tf[3]);                  // translation column
    glm::vec3 forward = glm::normalize(glm::vec3(tf[0]));  // local +X in world
    glm::vec3 up      = glm::normalize(glm::vec3(tf[2]));  // local +Z in world
    return glm::lookAt(pos, pos + forward, up);
  }

  glm::mat4 getProjMat() { return proj_mat_; }

  // Where and under what frame this sensor's data is published. The sensor holds
  // these as opaque strings; the transform-tree code owns how they are named and
  // hands them in, so the sensor stays decoupled from drones and frame naming.
  void setTopic(std::string topic) { topic_ = std::move(topic); }
  void setFrameId(std::string frame_id) { frame_id_ = std::move(frame_id); }

  // The published frame uniquely identifies this sensor, so it doubles as the
  // handoff key that keeps only the newest frame per sensor.
  [[nodiscard]] const std::string& sensorKey() const { return frame_id_; }

  // The entity this sensor is mounted on. Lets the tree code read the sensor's
  // name and owning vehicle without the sensor knowing anything about either.
  [[nodiscard]] const Entity& entity() const { return *parent_; }

  // Gate is created on first use so it starts from a live clock, not load time.
  bool captureDue(std::chrono::steady_clock::time_point now) {
    if (!gate_) {
      gate_ = std::make_unique<RateGate>(settings_.rate_hz, now);
    }
    return gate_->due(now);
  }

  // Assumes update() already rendered depth_tex_ this frame.
  void captureDepth() {
    if (free_pbos_.empty()) {
      return;  // all buffers in flight; a fresh frame comes next tick anyway
    }

    const int w = static_cast<int>(settings_.width);
    const int h = static_cast<int>(settings_.height);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, mm_fbo_.get());
    glViewport(0, 0, w, h);
    glDisable(GL_DEPTH_TEST);

    glUseProgram(convert_shader_->getID());
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depth_tex_.get());
    convert_shader_->setUniformInt("depth_tex", 0);
    convert_shader_->setUniformVec2(
        "z_range", glm::vec2(settings_.z_near, settings_.z_far));

    glBindVertexArray(vao_.get());
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    const int pbo = free_pbos_.back();
    free_pbos_.pop_back();

    // tight-pack rows so the readback is exactly w*h*2 bytes even for odd widths
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[pbo].get());
    glReadPixels(0, 0, w, h, GL_RED_INTEGER, GL_UNSIGNED_SHORT, nullptr);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);

    GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    in_flight_.push_back({pbo, fence, buildHeader()});

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glEnable(GL_DEPTH_TEST);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  // Polls without blocking, so calling it every frame never stalls the render
  // thread; the oldest readback finishes first, hence front().
  bool tryReadback(SensorFrame& out) {
    if (in_flight_.empty()) {
      return false;
    }

    InFlight&    front  = in_flight_.front();
    const GLenum status = glClientWaitSync(front.fence, 0, 0);
    if (status != GL_ALREADY_SIGNALED && status != GL_CONDITION_SATISFIED) {
      return false;
    }

    const std::size_t bytes =
        static_cast<std::size_t>(settings_.width) * settings_.height *
        sizeof(uint16_t);

    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo_[front.pbo].get());
    const void* mapped = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    const bool  ok     = mapped != nullptr;
    if (ok) {
      out.payload.resize(bytes);
      std::memcpy(out.payload.data(), mapped, bytes);
      glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    out.topic  = topic_;
    out.header = std::move(front.header);

    glDeleteSync(front.fence);
    free_pbos_.push_back(front.pbo);
    in_flight_.pop_front();

    return ok;
  }

 private:
  void setProjectionMatrix() {
    proj_mat_[0][0] = 2 * settings_.fx / settings_.width;
    proj_mat_[2][0] = 1 - (2 * settings_.cx / settings_.width);
    proj_mat_[1][1] = 2 * settings_.fy / settings_.height;
    proj_mat_[2][1] = (2 * settings_.cy / settings_.height) - 1;
    proj_mat_[2][2] = -(settings_.z_far + settings_.z_near) /
                      (settings_.z_far - settings_.z_near);
    proj_mat_[2][3] = -1.F;
    proj_mat_[3][2] = -2 * settings_.z_far * settings_.z_near /
                      (settings_.z_far - settings_.z_near);
    proj_mat_[3][3] = 0.F;
  }

  // Builds the R16UI target, the triple-buffered PBOs and the convert shader.
  void initReadback() {
    const int w = static_cast<int>(settings_.width);
    const int h = static_cast<int>(settings_.height);

    convert_shader_ = std::make_unique<Shader>(depth_convert_vertex_shader,
                                               depth_convert_fragment_shader);

    glGenTextures(1, mm_tex_.addr());
    glBindTexture(GL_TEXTURE_2D, mm_tex_.get());
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16UI, w, h, 0, GL_RED_INTEGER,
                 GL_UNSIGNED_SHORT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenFramebuffers(1, mm_fbo_.addr());
    glBindFramebuffer(GL_FRAMEBUFFER, mm_fbo_.get());
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           mm_tex_.get(), 0);
    GLenum draw_buffer = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &draw_buffer);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      throw std::runtime_error("[GPU Sensor] mm framebuffer incomplete");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    const std::size_t bytes =
        static_cast<std::size_t>(w) * h * sizeof(uint16_t);
    for (auto& pbo : pbo_) {
      glGenBuffers(1, pbo.addr());
      glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo.get());
      glBufferData(GL_PIXEL_PACK_BUFFER, bytes, nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    free_pbos_.clear();
    for (int i = 0; i < kNumPbos; ++i) {
      free_pbos_.push_back(i);
    }
  }

  [[nodiscard]] std::string buildHeader() const {
    volasim_msgs::DepthCamera msg;

    volasim_msgs::Header* header = msg.mutable_header();
    header->set_stamp_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    // The cloud is built in the optical convention (z forward), so the tree code
    // hands this sensor its optical frame as frame_id_.
    header->set_frame_id(frame_id_);

    msg.set_width(static_cast<uint32_t>(settings_.width));
    msg.set_height(static_cast<uint32_t>(settings_.height));
    msg.set_encoding(volasim_msgs::DepthCamera::DEPTH_U16_MM);
    msg.set_fx(settings_.fx);
    msg.set_fy(settings_.fy);
    msg.set_cx(settings_.cx);
    msg.set_cy(settings_.cy);
    msg.set_z_near(settings_.z_near);
    msg.set_z_far(settings_.z_far);

    std::string out;
    if (!msg.SerializeToString(&out)) {
      std::cerr << "[GPUSensor] failed to serialize DepthCamera header\n";
    }
    return out;
  }

 private:
  static constexpr int kNumPbos = 3;

  // fence signals when the PBO's DMA is done and safe to map.
  struct InFlight {
    int         pbo;
    GLsync      fence;
    std::string header;
  };

  Settings settings_;

  GLResource<FboDeleter> fbo_;
  GLResource<TexDeleter> depth_tex_;
  GLResource<VaoDeleter> vao_;

  // readback path: depth -> uint16 mm color target -> triple-buffered PBOs
  GLResource<FboDeleter>                          mm_fbo_;
  GLResource<TexDeleter>                          mm_tex_;
  std::array<GLResource<BufferDeleter>, kNumPbos> pbo_;
  std::unique_ptr<Shader>                         convert_shader_;
  std::vector<int>                                free_pbos_;
  std::deque<InFlight>                            in_flight_;

  // Opaque publish labels, assigned by the transform-tree code after load.
  std::string topic_{"depth"};
  std::string frame_id_{"sensor"};

  // paces this sensor's captures at settings_.rate_hz; lazily created
  std::unique_ptr<RateGate> gate_;

  Entity* parent_;
  // zero-initialized: setProjectionMatrix() only writes the nonzero cells and
  // relies on the rest being 0 (GLM's default ctor leaves them uninitialized).
  glm::mat4 proj_mat_ = glm::mat4(0.0F);
  glm::mat4 sensor_view_mat_;

  std::unique_ptr<Shader> point_shader_;
  std::unique_ptr<Shader> depth_shader_;

  int num_points_ = 0;
};

#endif
