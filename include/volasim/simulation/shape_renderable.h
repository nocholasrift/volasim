#ifndef SHAPERENDERABLE_H
#define SHAPERENDERABLE_H

#include <volasim/simulation/renderable.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <string_view>

struct ShapeMetadata {
  ShapeType type = ShapeType::kUndefined;

  glm::vec3 pos = glm::vec3(0., 0., 0.);
  glm::quat rot = glm::quat(1., 0., 0., 0.);
  // glm::vec3 color = glm::vec3(1., 0., 0.);
  std::string color = "#CCCCCC";

  std::string name = "";

  float size = 0.5;

  float radius = 0.5;
  float height = 0.5;

  int slices = 32;
  int stacks = 2;

  // for plane
  float x_min = -1.;
  float y_min = -1.;
  float x_max = 1.;
  float y_max = 1.;
  float z = 0.;

  GLuint vao = 0;
  GLuint vbo = 0;
  GLuint ebo = 0;
  GLsizei index_count = 0;
};

class ShapeRenderable : public Renderable {
 public:
  ShapeRenderable() {}
  ShapeRenderable(const ShapeMetadata& meta);
  virtual ~ShapeRenderable();

  void draw(Shader& shader) override;

  virtual ShapeType getType() const override { return meta_.type; }
  ShapeMetadata getShapeMeta() const { return meta_; }

  virtual void buildFromXML(const pugi::xml_node& item) override;

 private:
  ShapeType type_;

  void setup(ShapeType type);

  glm::vec3 hexToRGB(std::string_view hex_str);

  ShapeMetadata meta_;

  /*GLUquadric* quad = nullptr;*/

  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;

  std::unordered_map<std::string_view, ShapeType> shape_map_ = {
      {"sphere", ShapeType::kSphere},
      {"cylinder", ShapeType::kCylinder},
      {"cube", ShapeType::kCube},
      {"plane", ShapeType::kPlane}};
};

#endif
