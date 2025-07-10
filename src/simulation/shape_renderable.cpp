#include <GL/glu.h>
#include <GL/glut.h>
#include <volasim/simulation/shape_renderable.h>

ShapeRenderable::ShapeRenderable(ShapeType type, const ShapeMetadata& meta)
    : type_(type), meta_(meta) {}

ShapeRenderable::~ShapeRenderable() {}

void ShapeRenderable::draw() {
  switch (type_) {

    case ShapeType::kSphere:
      glutSolidSphere(meta_.scale, meta_.slices, meta_.stacks);
      break;
    case ShapeType::kCube:
      glutSolidCube(meta_.scale);
      break;
    case ShapeType::kCylinder:
      drawCylinder();
      break;
    case ShapeType::kPlane:
      break;
  }
}

void ShapeRenderable::drawCylinder() {
  static GLUquadric* quad = gluNewQuadric();
  gluCylinder(quad, meta_.scale, meta_.scale, meta_.height, meta_.slices,
              meta_.stacks);
}
