#ifndef RENDERABLE_H
#define RENDERABLE_H

class Renderable {
 public:
  virtual ~Renderable() = default;
  virtual void draw() = 0;

 private:
};

#endif
