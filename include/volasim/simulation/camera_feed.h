#ifndef VOLASIM_CAMERA_FEED_H
#define VOLASIM_CAMERA_FEED_H

#include <volasim/simulation/gl_resource.h>

#include <string>

class Entity;
class GPUSensor;
class Shader;
class WorldSnapshot;

// A color render of the scene from a sensor's pose, into an offscreen texture
// the GUI can show. Render thread only; needs a live GL context to construct.
class CameraFeed {
 public:
  // The feed keeps the sensor's aspect ratio, scaled to `width` pixels across.
  CameraFeed(const GPUSensor& sensor, int width);

  // Assumes `shader` is bound with its lighting uniforms already set.
  void render(const Entity& world, const WorldSnapshot& snapshot,
              Shader& shader) const;

  [[nodiscard]] const std::string& name() const;
  [[nodiscard]] GLuint             texture() const { return color_.get(); }
  [[nodiscard]] int                width() const { return width_; }
  [[nodiscard]] int                height() const { return height_; }

  // Set by the GUI while the feed's window is showing; render() is skipped
  // otherwise, so a closed feed costs nothing.
  bool open{false};

 private:
  const GPUSensor* sensor_;
  int              width_;
  int              height_;

  GLResource<FboDeleter>          fbo_;
  GLResource<TexDeleter>          color_;
  GLResource<RenderbufferDeleter> depth_;
};

#endif
