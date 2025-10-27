#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <volasim/simulation/shader.h>

enum class ShapeType { kUndefined = 0, kSphere, kCube, kCylinder, kPlane };

class Renderable {
 public:
  virtual ~Renderable() = default;
  virtual void draw(Shader& shader) = 0;
  virtual ShapeType getType() const = 0;
  virtual void createBuffer() = 0;

 private:
};

#endif
