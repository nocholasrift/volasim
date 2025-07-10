#ifndef SHAPERENDERABLE_H
#define SHAPERENDERABLE_H

#include <volasim/simulation/renderable.h>
#include <glm/glm.hpp>

enum class ShapeType { kSphere = 0, kCube, kCylinder, kPlane };

struct ShapeMetadata {
  double scale;
  double height;
  int slices;
  int stacks;
};

class ShapeRenderable : public Renderable {
 public:
  ShapeRenderable(ShapeType type, const ShapeMetadata& meta);
  virtual ~ShapeRenderable();

  void draw() override;

  void drawCylinder();

 private:
  ShapeType type_;

  ShapeMetadata meta_;
};

#endif
