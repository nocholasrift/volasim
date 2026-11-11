#include <volasim/sensors/depth_sensor.h>
#include <volasim/simulation/camera_feed.h>

#include <cmath>
#include <stdexcept>

CameraFeed::CameraFeed(const GPUSensor& sensor, int width)
    : sensor_(&sensor),
      width_(width),
      height_(static_cast<int>(std::lround(static_cast<double>(width) *
                                           sensor.height() / sensor.width()))) {
  glGenTextures(1, color_.addr());
  glBindTexture(GL_TEXTURE_2D, color_.get());
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  glGenRenderbuffers(1, depth_.addr());
  glBindRenderbuffer(GL_RENDERBUFFER, depth_.get());
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  glGenFramebuffers(1, fbo_.addr());
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_.get());
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         color_.get(), 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, depth_.get());

  const bool complete =
      glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  if (!complete) {
    throw std::runtime_error("[CameraFeed] framebuffer incomplete");
  }
}

void CameraFeed::render(const Entity& world, const WorldSnapshot& snapshot,
                        Shader& shader) const {
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  glBindFramebuffer(GL_FRAMEBUFFER, fbo_.get());
  glViewport(0, 0, width_, height_);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  world.draw(snapshot, glm::mat4(1.F), sensor_->getViewMat(snapshot),
             sensor_->getProjMat(), shader);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

const std::string& CameraFeed::name() const {
  return sensor_->name();
}
