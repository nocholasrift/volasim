#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <volasim/simulation/shader.h>

#include <pugixml.hpp>

#include <memory>
#include <string_view>
#include <unordered_map>

enum class ShapeType {
  kUndefined = 0,
  kSphere,
  kCube,
  kCylinder,
  kPlane,
  kMesh
};

class Renderable {
 public:
  Renderable() {}
  virtual ~Renderable() = default;
  virtual void draw(Shader& shader) = 0;
  virtual ShapeType getType() const = 0;
  virtual void buildFromXML(const pugi::xml_node& item) = 0;

 private:
};

#endif
