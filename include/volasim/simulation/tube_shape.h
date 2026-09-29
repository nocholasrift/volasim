#ifndef VOLASIM_TUBE_SHAPE_H
#define VOLASIM_TUBE_SHAPE_H

#include <volasim/simulation/overlay_drawable.h>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

struct TrajectoryData {
  std::vector<glm::vec3> points;
  glm::vec3              color{0.f, 1.f, 0.f};
  float                  radius{0.03f};
  std::string            frame_id;
};

class TubeShape : public OverlayDrawable {
 public:
  TubeShape(GLuint shared_vao, GLsizei shared_index_count);

  void update(TrajectoryData data);

  void draw(const glm::mat4& view, const glm::mat4& proj,
            const glm::mat4& frame_transform, Shader& shader) override;

  [[nodiscard]] const std::string& frameId() const { return data_.frame_id; }

 private:
  GLuint  cylinder_vao_;
  GLsizei cylinder_index_count_;

  TrajectoryData data_;
};

#endif
