#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <volasim/simulation/shader.h>

class Renderable {
 public:
  virtual ~Renderable() = default;
  virtual void draw(Shader& shader) = 0;

 private:
};

#endif
