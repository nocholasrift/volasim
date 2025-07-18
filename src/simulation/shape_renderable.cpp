#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <volasim/simulation/shape_renderable.h>

ShapeRenderable::ShapeRenderable(ShapeType type, const ShapeMetadata& meta)
    : type_(type), meta_(meta) {
}

ShapeRenderable::~ShapeRenderable() {}

void ShapeRenderable::draw() {
  switch (type_) {

    case ShapeType::kSphere:
      glutSolidSphere(meta_.scale, meta_.slices, meta_.stacks);
      break;
    case ShapeType::kCube:
      glColor3f(1.0f, 0.0f, 0.0f);
      glutSolidCube(meta_.scale);
      break;
    case ShapeType::kCylinder:
      drawCylinder();
      break;
    case ShapeType::kPlane:
      glColor3f(0.5f, 0.5f, 0.5f);  // Gray ground
      drawGroundPlane();
      break;
  }
  glColor3f(1.0, 1.0, 1.0);
}

void ShapeRenderable::drawCylinder() {
  static GLUquadric* quad = gluNewQuadric();
  gluCylinder(quad, meta_.scale, meta_.scale, meta_.height, meta_.slices,
              meta_.stacks);
}

void ShapeRenderable::drawGroundPlane(float size) {
    // glDisable(GL_LIGHTING);  // Optional: disable lighting for flat color

    glBegin(GL_QUADS);
    glVertex3f(-size, -size, 0.0f);
    glVertex3f( size, -size, 0.0f);
    glVertex3f( size,  size, 0.0f);
    glVertex3f(-size,  size, 0.0f);
    glEnd();

    // glEnable(GL_LIGHTING);  // Re-enable lighting if needed
}
