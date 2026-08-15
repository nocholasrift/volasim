#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct Transform {
  glm::vec3 position{0.F};
  glm::quat rotation{1.F, 0.F, 0.F, 0.F};  // w,x,y,z
  glm::vec3 scale{1.F};

  [[nodiscard]] glm::mat4 toMatrix() const {
    glm::mat4 trans_mat = glm::translate(glm::mat4(1.0F), position);
    glm::mat4 rot_mat   = glm::mat4_cast(rotation);
    glm::mat4 scale_mat = glm::scale(glm::mat4(1.0F), scale);

    return trans_mat * rot_mat * scale_mat;
  }
};

#endif
