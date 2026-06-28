#ifndef DEPTHSENSOR_H
#define DEPTHSENSOR_H

#include <volasim/simulation/display_object.h>
#include <volasim/simulation/dynamic_object.h>
#include <volasim/simulation/shader.h>

#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

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
    "  vec2 uv = vec2(float(j) / sensor_size.x, float(row) / sensor_size.y);\n"
    "  float d = texture(depth_tex, uv).r;\n"
    "  if (d >= 1.0) {\n"
    "    gl_Position = vec4(0.0, 0.0, 2.0, 1.0);\n"  // behind far plane — clipped
    "    gl_PointSize = 0.0;\n"
    "    return;\n"
    "  }\n"
    "  float x_ndc = 2.0 * float(j) / sensor_size.x - 1.0;\n"
    "  float y_ndc = 2.0 * float(row) / sensor_size.y - 1.0;\n"
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

enum class DepthSensorType { kLidar = 0, kDepthCamera };

struct DepthSensorSettings {
  float width;
  float height;
  float fx, fy;
  float cx, cy;
  float z_near, z_far;

  DepthSensorType type;

  glm::vec3 pos = glm::vec3(0, 0, 0);
  glm::quat rot = glm::quat(1., 0., 0., 0.);
};

class GPUSensor {
 public:
  GPUSensor(const DepthSensorSettings& settings, DynamicObject* parent) {
    if (parent == nullptr)
      throw std::runtime_error("[Depth Sensor] Parent is null!");

    settings_ = settings;

    if (settings_.width == 0 || settings_.height == 0)
      throw std::invalid_argument(
          "[DepthSensorSettings] width/height must be > 0");
    if (settings_.fx <= 0 || settings_.fy <= 0)
      throw std::invalid_argument("[DepthSensorSettings] fx/fy must be > 0");
    if (settings_.z_near <= 0 || settings_.z_far <= settings_.z_near)
      throw std::invalid_argument(
          "[DepthSensorSettings] z_near > 0 and z_far > z_near");

    parent_ = parent;
    num_points_ = settings_.width * settings_.height;
    setProjectionMatrix();
  }

  ~GPUSensor() {
    if (fbo_) {
      glDeleteFramebuffers(1, &fbo_);
    }

    if (depth_tex_) {
      glDeleteTextures(1, &depth_tex_);
    }
    if (vao_) {
      glDeleteVertexArrays(1, &vao_);
    }
  }

  void init() {
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    glGenTextures(1, &depth_tex_);
    glBindTexture(GL_TEXTURE_2D, depth_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, settings_.width,
                 settings_.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           depth_tex_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      throw std::runtime_error(
          "[Depth Sensor] Framebuffer initialization failed!");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // empty VAO — point shader uses gl_VertexID, no vertex attributes needed
    glGenVertexArrays(1, &vao_);

    point_shader_ =
        std::make_unique<Shader>(point_vertex_shader, point_fragment_shader);
    depth_shader_ =
        std::make_unique<Shader>(depth_vertex_shader, depth_fragment_shader);
  }

  // Renders the scene into the sensor's depth FBO. Call once per frame before draw().
  void update(DisplayObject* world, Shader& shader) {
    if (world == nullptr)
      throw std::invalid_argument("[GPU Sensor] world is null");

    sensor_view_mat_ = getViewMat();

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, settings_.width, settings_.height);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);

    glUseProgram(depth_shader_->getID());
    world->draw(sensor_view_mat_, proj_mat_, *depth_shader_);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
  }

  // Draws the point cloud using the main camera's view/proj matrices.
  void draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
            Shader& shader) {
    glUseProgram(point_shader_->getID());

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depth_tex_);
    point_shader_->setUniformInt("depth_tex", 0);
    point_shader_->setUniformMat4("sensor_inv_vp",
                                  glm::inverse(proj_mat_ * sensor_view_mat_));
    point_shader_->setUniformMat4("main_mvp", proj_mat * view_mat);
    point_shader_->setUniformVec2("sensor_size",
                                  glm::vec2(settings_.width, settings_.height));

    glDepthFunc(GL_LEQUAL);
    glEnable(GL_POLYGON_OFFSET_POINT);
    glPolygonOffset(-1.0f, -1.0f);
    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, num_points_);
    glBindVertexArray(0);
    glDisable(GL_POLYGON_OFFSET_POINT);
    glDepthFunc(GL_LESS);

    glBindTexture(GL_TEXTURE_2D, 0);
  }

  glm::mat4 getViewMat() {
    glm::quat rot = parent_->getRotation();
    glm::vec3 pos = parent_->getTranslation();
    glm::vec3 forward = rot * glm::vec3(1, 0, 0);  // sensor looks down drone +X
    glm::vec3 up = rot * glm::vec3(0, 0, 1);       // drone +Z is up
    return glm::lookAt(pos, pos + forward, up);
  }

  glm::mat4 getProjMat() { return proj_mat_; }

 private:
  void setProjectionMatrix() {
    proj_mat_[0][0] = 2 * settings_.fx / settings_.width;
    proj_mat_[2][0] = 1 - 2 * settings_.cx / settings_.width;
    proj_mat_[1][1] = 2 * settings_.fy / settings_.height;
    proj_mat_[2][1] = 2 * settings_.cy / settings_.height - 1;
    proj_mat_[2][2] = -(settings_.z_far + settings_.z_near) /
                      (settings_.z_far - settings_.z_near);
    proj_mat_[2][3] = -1;
    proj_mat_[3][2] = -2 * settings_.z_far * settings_.z_near /
                      (settings_.z_far - settings_.z_near);
    proj_mat_[3][3] = 0.f;
  }

 private:
  DepthSensorSettings settings_;

  GLuint fbo_ = 0;
  GLuint depth_tex_ = 0;
  GLuint vao_ = 0;

  DynamicObject* parent_;
  glm::mat4 proj_mat_;
  glm::mat4 sensor_view_mat_;

  std::unique_ptr<Shader> point_shader_;
  std::unique_ptr<Shader> depth_shader_;

  int num_points_ = 0;
};

#endif
