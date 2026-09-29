#include <glad/glad.h>

#include <volasim/simulation/tube_shape.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include <iostream>
#include <utility>

TubeShape::TubeShape(GLuint shared_vao, GLsizei shared_index_count)
    : cylinder_vao_(shared_vao), cylinder_index_count_(shared_index_count) {}

void TubeShape::update(TrajectoryData data) {
  data_ = std::move(data);
}

void TubeShape::draw(const glm::mat4& view, const glm::mat4& proj,
                     const glm::mat4& frame_transform, Shader& shader) {
  if (data_.points.size() < 2) {
    return;
  }

  static int draw_count = 0;
  if (draw_count < 3) {
    std::cout << "[TubeShape] draw() #" << draw_count
              << ": pts=" << data_.points.size() << " vao=" << cylinder_vao_
              << " idx=" << cylinder_index_count_ << " r=" << data_.radius
              << "\n";
    const auto& p0 = data_.points[0];
    const auto& p1 = data_.points[1];
    std::cout << "  p0=(" << p0.x << "," << p0.y << "," << p0.z << ")"
              << " p1=(" << p1.x << "," << p1.y << "," << p1.z << ")\n";
    std::cout << "  frame_tf[3]=(" << frame_transform[3][0] << ","
              << frame_transform[3][1] << "," << frame_transform[3][2] << ")\n";
    ++draw_count;
  }

  shader.setUniformVec3("color", data_.color);
  glBindVertexArray(cylinder_vao_);

  for (size_t i = 0; i + 1 < data_.points.size(); ++i) {
    glm::vec3 dir = data_.points[i + 1] - data_.points[i];
    float     len = glm::length(dir);
    if (len < 1e-6f) {
      continue;
    }

    glm::quat rot = glm::rotation(glm::vec3(0.f, 0.f, 1.f), dir / len);
    glm::mat4 segment =
        glm::translate(glm::mat4(1.f), data_.points[i]) * glm::mat4_cast(rot) *
        glm::scale(glm::mat4(1.f), glm::vec3(data_.radius, data_.radius, len));

    glm::mat4 model = frame_transform * segment;
    glm::mat4 mvp   = proj * view * model;

    shader.setUniformMat4("mvp", mvp);
    shader.setUniformMat4("model", model);

    glDrawElements(GL_TRIANGLES, cylinder_index_count_, GL_UNSIGNED_INT,
                   (void*)0);
  }

  glBindVertexArray(0);
}
