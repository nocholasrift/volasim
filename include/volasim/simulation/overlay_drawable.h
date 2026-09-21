#ifndef VOLASIM_OVERLAY_DRAWABLE_H
#define VOLASIM_OVERLAY_DRAWABLE_H

#include <volasim/simulation/shader.h>

#include <glm/glm.hpp>

class OverlayDrawable {
 public:
  virtual ~OverlayDrawable() = default;

  virtual void draw(const glm::mat4& view, const glm::mat4& proj,
                    const glm::mat4& frame_transform, Shader& shader) = 0;

  bool visible{true};
};

#endif
