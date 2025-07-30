#ifndef DEPTHSENSOR_H
#define DEPTHSENSOR_H

#include <volasim/simulation/display_object.h>
#include <volasim/simulation/shader.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <memory>
#include <stdexcept>
#include <string>

// shader code
static const std::string vertex_shader =
    "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "uniform mat4 uMPV;\n"
    "void main(){\n"
    "gl_Position = uMPV * vec4(aPos, 1.0);\n"
    "}\n";

static const std::string fragment_shader =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "FragColor = vec4(0.0f, 1.0f, 0.0f);\n"
    "}\n";

enum class DepthSensorType { kLidar = 0, kDepthCamera };

struct DepthSensorSettings {
  double fov;
  double vertical_resolution;
  double horizontal_resolution;

  DepthSensorType type;

  glm::vec3 pos = glm::vec3(0, 0, 0);
  glm::quat rot = glm::quat(1., 0., 0., 0.);
};

class GPUSensor {
 public:
  GPUSensor(const DepthSensorSettings& settings, DisplayObject* parent) {
    if (parent == nullptr)
      throw std::runtime_error("[Depth Sensor] Parent is null!");

    fov_ = settings.fov;
    vertical_resolution_ = settings.vertical_resolution;
    horizontal_resolution_ = settings.horizontal_resolution;

    parent_ = parent;
  }
  // DepthSensor(std::string id) {}

  ~GPUSensor() {}

  void init() {
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    // create a depth texture
    GLuint depth_tex;
    glGenTextures(1, &depth_tex);

    // configure the texture
    glBindTexture(GL_TEXTURE_2D, depth_tex);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, 800, 600, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

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

    // setup shader program
    // depth_program_ =
    //     std::make_unique<DepthSensorShader>(vertex_shader, fragment_shader);
  }

  void update(DisplayObject* world) {
    if (world == nullptr)
      throw std::invalid_argument(
          "[GPU Sensor] Passed in Display Object is null");

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_GREATER);
    glClearDepth(0.f);
    glClear(GL_DEPTH_BUFFER_BIT);

    glUseProgram(depth_program_->getID());

    world->draw();

    float* data;
    glReadPixels(0, 0, 800, 600, GL_DEPTH_COMPONENT, GL_FLOAT, data);
  }

 private:
  double fov_;
  double vertical_resolution_;
  double horizontal_resolution_;

  unsigned int fbo_;

  // std::unique_ptr<DepthSensorShader> depth_program_;

  DisplayObject* parent_;
};

#endif
