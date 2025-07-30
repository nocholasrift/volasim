#ifndef SHAPERENDERABLE_H
#define SHAPERENDERABLE_H

#include <volasim/simulation/renderable.h>

#include <GL/glu.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <string_view>

enum class ShapeType { kSphere = 0, kCube, kCylinder, kPlane };

struct ShapeMetadata {
  ShapeType type = ShapeType::kSphere;

  glm::vec3 pos = glm::vec3(0., 0., 0.);
  glm::quat rot = glm::quat(1., 0., 0., 0.);
  // glm::vec3 color = glm::vec3(1., 0., 0.);
  std::string color = "#CCCCCC";

  std::string name = "";

  double size = 0.5;

  double radius = 0.5;
  double height = 0.5;

  int slices = 32;
  int stacks = 2;

  // for plane
  double x_min = -1.;
  double y_min = -1.;
  double x_max = 1.;
  double y_max = 1.;
  double z = 0.;

  GLuint vao = 0;
  GLuint vbo = 0;
  GLuint ebo = 0;
  GLsizei index_count = 0;
};

class ShapeRenderable : public Renderable {
 public:
  ShapeRenderable(ShapeType type, const ShapeMetadata& meta);
  virtual ~ShapeRenderable();

  void draw() override;

  ShapeType getType() const { return type_; }
  ShapeMetadata getShapeMeta() const { return meta_; }

 private:
  ShapeType type_;

  void setup(ShapeType type);

  void drawCylinder();
  void drawGroundPlane();

  glm::vec3 hexToRGB(std::string_view hex_str);

  ShapeMetadata meta_;

  GLUquadric* quad = nullptr;

  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;
};

#endif
