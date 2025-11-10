#ifndef DEPTHSENSOR_H
#define DEPTHSENSOR_H

#include <volasim/simulation/display_object.h>
#include <volasim/simulation/shader.h>

#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// shader code
// static const std::string vertex_shader =
//     "#version 330 core\n"
//     "layout (location = 0) in vec3 aPos;\n"
//     "uniform mat4 mvp;\n"
//     "void main(){\n"
//     "gl_Position = mvp * vec4(aPos, 1.0);\n"
//     "}\n";

static const std::string vertex_shader =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 model;\n"
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "gl_Position = mvp * vec4(aPos, 1.0f);\n"
    "}\n";

static const std::string fragment_shader =
    "#version 330 core\n"
    "float near = .25;\n"
    "float far = 10.0;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "float depth = gl_FragCoord.z;\n"
    "float ndc = depth * 2.0- 1.0;\n"
    "float linearDepth = (2.0 * near * far)/(far + near - ndc * (far - "
    "near))\n;"
    "float normDepth = (linearDepth - near) / (far - near)\n;"
    "FragColor = vec4(vec3(normDepth), 1.0);\n"
    "}\n";

static const std::string point_vertex_shader =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 inv_p;\n"
    "void main(){\n"
    // "vec4 viewSpacePos = inv_p * vec4(aPos, 1.0);\n"
    // "viewSpacePos = viewSpacePos / viewSpacePos.w;\n"
    // "gl_Position = mvp * viewSpacePos;\n"
    "gl_Position = vec4(aPos, 1.0);\n"
    "gl_PointSize = 3.0;\n"
    "}\n";

static const std::string point_fragment_shader =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "FragColor = vec4(0.0f, 0.0f, 1.0f, 1.0f);\n"
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
  GPUSensor(const DepthSensorSettings& settings, DisplayObject* parent) {
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

    depth_data_.resize(num_points_);
    pcl_.resize(num_points_ * 3);

    setProjectionMatrix();
  }

  ~GPUSensor() {
    if (fbo_)
      glDeleteFramebuffers(1, &fbo_);
    if (vao_)
      glDeleteVertexArrays(1, &vao_);
    if (vbo_)
      glDeleteBuffers(1, &vbo_);
  }

  void init() {
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    // create a depth texture
    GLuint depth_tex;
    glGenTextures(1, &depth_tex);

    // configure the texture
    glBindTexture(GL_TEXTURE_2D, depth_tex);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, settings_.width,
                 settings_.height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
    //
    // attach it to currently bound framebuffer object
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           depth_tex, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      throw std::runtime_error(
          "[Depth Sensor] Framebuffer initialization failed!");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    // setup shader program
    point_shader_ =
        std::make_unique<Shader>(point_vertex_shader, point_fragment_shader);

    depth_shader_ = std::make_unique<Shader>(vertex_shader, fragment_shader);
  }

  void update(DisplayObject* world, Shader& shader) {
    if (world == nullptr)
      throw std::invalid_argument(
          "[GPU Sensor] Passed in Display Object is null");

    // glm::mat4 transform = parent_->getLocalTransform();
    // glm::mat4 flip_x =
    //     glm::rotate(glm::mat4(1.0f), glm::pi<float>(), glm::vec3(1, 0, 0));
    // glm::mat4 view_mat = glm::inverse(transform * flip_x);
    glm::mat4 view_mat = getViewMat();

    // for (int i = 0; i < 4; ++i) {
    //   std::cout << view_mat[0][i] << " " << view_mat[1][i] << " "
    //             << view_mat[2][i] << " " << view_mat[3][i] << "\n";
    // }
    // std::cout << std::endl;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glEnable(GL_DEPTH_TEST);
    // glDepthFunc(GL_GREATER);
    // glClearDepth(0.f);
    glClear(GL_DEPTH_BUFFER_BIT);

    // glUseProgram(shader.getID());
    glUseProgram(depth_shader_->getID());
    world->draw(view_mat, proj_mat_, *depth_shader_);

    // glUseProgram(depth_shader_->getID());
    // world->draw(view_mat, proj_mat_, *depth_shader_);

    glReadPixels(0, 0, settings_.width, settings_.height, GL_DEPTH_COMPONENT,
                 GL_FLOAT, depth_data_.data());

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // glDepthFunc(GL_LESS);
    // glClearDepth(1.f);

    // convert to linear depth from clip space
    // const float fn = settings_.z_far * settings_.z_near;
    // const float dz = settings_.z_far - settings_.z_near;
    // for (float& depth : depth_data_) {
    // if (depth != 0.f)
    //   depth = fn / (depth * dz + settings_.z_near);
    // depth = 2 * depth - 1;
    // depth = 2 * fn / (settings_.z_far + settings_.z_near - depth * dz);
    // }

    /*std::vector<float> pcl;*/
    /*pcl.resize(settings_.width * settings_.height * 3);*/
    // -- -debug NDC / unprojection check-- -
    auto dump_sample = [&](int i, int j) {
      int gl_row =
          i;  // if you think readpixels already bottom-left -> use i; if your depth_data_ was filled top-left use gl_row = height - 1 - i
      float d = depth_data_[gl_row * settings_.width + j];  // in [0,1]
      float x_ndc = (2.0f * j / settings_.width) - 1.0f;
      float y_ndc = (2.0f * i / settings_.height) - 1.0f;
      float z_ndc = 2.0f * d - 1.0f;

      const float fn = settings_.z_far * settings_.z_near;
      const float dz = settings_.z_far - settings_.z_near;
      float distance = 2 * fn / (settings_.z_far + settings_.z_near - d * dz);

      glm::vec4 ndc(x_ndc, y_ndc, z_ndc, 1.0f);
      glm::mat4 invP = glm::inverse(proj_mat_);
      glm::vec4 cam = invP * ndc;
      cam /= cam.w;
      glm::mat4 invV = glm::inverse(
          view_mat);  // view_mat used when rendering the depth buffer (depth sensor view)
      glm::vec4 world = invV * cam;

      std::cout << "\nPIX(" << i << "," << j << ") gl_row=" << gl_row
                << " depth=" << d << " ndc=(" << x_ndc << "," << y_ndc << ","
                << z_ndc << ")"
                << " cam=(" << cam.x << "," << cam.y << "," << cam.z << ")"
                << " world=(" << world.x << "," << world.y << "," << world.z
                << ")\n";

      std::cout << "distance: " << distance << "\n";
    };

    // choose pixels: center and three corners
    int cx = settings_.width / 2, cy = settings_.height / 2;
    dump_sample(cy, cx);  // image center
    // dump_sample(0, 0);                     // top-left in image coordinates
    // dump_sample(settings_.height - 1, 0);  // bottom-left
    // dump_sample(0, settings_.width - 1);   // top-right
    // exit(0);

    size_t count = 0;
    for (int i = 0; i < settings_.height; ++i) {
      for (int j = 0; j < settings_.width; ++j) {
        // float distance = depth_data_[i * settings_.width + j];
        // float x = (j - settings_.cx) * distance / settings_.fx;
        // float y = (i - settings_.cy) * distance / settings_.fy;
        // float z = distance;

        // float x = 2 * j / settings_.width - 1;
        // float y = 1.0 - (2.0 * i) / settings_.height;

        float x = 2.0 * j / settings_.width - 1;
        float y = 2.0 * i / settings_.height - 1;
        float z = 2 * depth_data_[i * settings_.width + j] - 1;

        pcl_[count++] = x;
        pcl_[count++] = y;
        pcl_[count++] = z;
      }
    }

    // std::cout << "pos count is: " << pos_count << " / " << count / 3 << "\n";
    // exit(0);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, pcl_.size() * sizeof(float), pcl_.data(),
                 GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  void draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
            Shader& shader) {

    glUseProgram(point_shader_->getID());

    glm::mat4 rot = glm::mat4_cast(parent_->getRotation());
    glm::mat4 trans =
        glm::translate(glm::mat4(1.0f), parent_->getTranslation());

    glm::mat4 full_pcl_transform =
        proj_mat * view_mat * glm::inverse(getViewMat());  // * trans * rot;
    point_shader_->setUniformMat4("mvp", full_pcl_transform);
    point_shader_->setUniformMat4("inv_p", glm::inverse(getProjMat()));

    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, num_points_);
    glBindVertexArray(0);
  }

  const std::vector<float>& getPointCloud() { return pcl_; }

  glm::mat4 getViewMat() {

    // glm::vec3 forward = getRotation() * glm::vec3(0., 0., -1.);
    glm::vec3 forward =
        glm::normalize(glm::vec3(-2.25, 1.5, 2.) - glm::vec3(-2.25, 6., 2.));
    glm::vec3 right = glm::cross(glm::vec3(0, 0, 1), forward);
    glm::vec3 up = glm::cross(forward, right);

    glm::mat4 rot = glm::rotate(glm::mat4(1.0f), -glm::half_pi<float>(),
                                glm::vec3(1, 0, 0));
    return rot *
           glm::lookAt(glm::vec3(-2.25, 1.5, 2.), glm::vec3(-2.25, 6., 2.), up);

    return glm::lookAt(glm::vec3(-2.25, 1.5, 2.), glm::vec3(-2.25, 6., 2.), up);
    // return glm::lookAt(parent_->getTranslation(),
    //                    parent_->getTranslation() + forward, up);

    // glm::mat4 transform = parent_->getLocalTransform();
    // glm::mat4 flip_x =
    //     glm::rotate(glm::mat4(1.0f), glm::pi<float>(), glm::vec3(1, 0, 0));
    // return glm::inverse(transform * flip_x);
  }

  glm::mat4 getProjMat() { return proj_mat_; }

 private:
  void setProjectionMatrix() {
    // proj_mat_[0][0] = 2 * settings_.fx / settings_.width;
    // proj_mat_[1][1] = 2 * settings_.fy / settings_.height;
    // proj_mat_[0][2] = 1 - 2 * settings_.cx / settings_.width;
    // proj_mat_[1][2] = 2 * settings_.cy / settings_.height - 1;
    // proj_mat_[2][2] = settings_.z_near / (settings_.z_far - settings_.z_near);
    // proj_mat_[3][2] = -1;
    //
    // proj_mat_[2][3] = settings_.z_far * settings_.z_near /
    //                   (settings_.z_far - settings_.z_near);

    proj_mat_[0][0] = 2 * settings_.fx / settings_.width;
    // proj_mat_[0][2] = 1 - 2 * settings_.cx / settings_.width;
    proj_mat_[2][0] = 1 - 2 * settings_.cx / settings_.width;
    proj_mat_[1][1] = 2 * settings_.fy / settings_.height;
    // proj_mat_[1][2] = 2 * settings_.cy / settings_.height - 1;
    proj_mat_[2][1] = 2 * settings_.cy / settings_.height - 1;
    // proj_mat_[2][2] = settings_.z_near / (settings_.z_far - settings_.z_near);
    proj_mat_[2][2] = -(settings_.z_far + settings_.z_near) /
                      (settings_.z_far - settings_.z_near);
    proj_mat_[2][3] = -1;

    // proj_mat_[3][2] = -settings_.z_far * settings_.z_near /
    //                   (settings_.z_far - settings_.z_near);
    proj_mat_[3][2] = -2 * settings_.z_far * settings_.z_near /
                      (settings_.z_far - settings_.z_near);

    // proj_mat_ = glm::perspective(
    //     glm::radians(75.f),                                      // fov
    //     static_cast<float>(settings_.width) / settings_.height,  // aspect ratio
    //     settings_.z_near, settings_.z_far);  // near & far plane
  }

 private:
  DepthSensorSettings settings_;

  unsigned int fbo_;
  unsigned int vao_;
  unsigned int vbo_;

  std::vector<float> depth_data_;

  DisplayObject* parent_;
  glm::mat4 proj_mat_;

  std::unique_ptr<Shader> point_shader_;
  std::unique_ptr<Shader> depth_shader_;

  int num_points_ = 0;

  std::vector<float> pcl_;
};

#endif
