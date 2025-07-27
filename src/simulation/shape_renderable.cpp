#include <volasim/simulation/shape_renderable.h>

#include <GL/gl.h>
#include <GL/glut.h>

#include <array>
#include <iostream>
#include <stdexcept>

ShapeRenderable::ShapeRenderable(ShapeType type, const ShapeMetadata& meta)
    : type_(type), meta_(meta) {
  quad = gluNewQuadric();
}

ShapeRenderable::~ShapeRenderable() {}

void ShapeRenderable::draw() {
  glm::vec3 color = hexToRGB(meta_.color);
  glColor3f(color[0], color[1], color[2]);
  switch (type_) {
    case ShapeType::kSphere:
      glutSolidSphere(meta_.radius, meta_.slices, meta_.stacks);
      break;
    case ShapeType::kCube:
      glutSolidCube(meta_.size);
      break;
    case ShapeType::kCylinder:
      drawCylinder();
      break;
    case ShapeType::kPlane:
      drawGroundPlane();
      break;
  }
  glColor3f(1.0, 1.0, 1.0);
}

void ShapeRenderable::drawCylinder() {
  // static GLUquadric* quad = gluNewQuadric();
  gluCylinder(quad, meta_.radius, meta_.radius, meta_.height, meta_.slices,
              meta_.stacks);
}

glm::vec3 ShapeRenderable::hexToRGB(std::string_view hex_str) {
  if (hex_str[0] != '#' || hex_str.length() != 7) {
    std::string err_str =
        "[ShapeRenderable] Invalid color string! " + std::string(hex_str) +
        "\nShould be formatted as 6 hex digits preceeded by #";
    throw std::invalid_argument(err_str);
  }

  glm::vec3 ret;
  std::string_view str_r = hex_str.substr(1, 2);
  std::string_view str_g = hex_str.substr(3, 2);
  std::string_view str_b = hex_str.substr(5, 2);

  std::array<std::string_view, 3> strs = {str_r, str_g, str_b};

  auto hexToInt = [](char c) -> uint8_t {
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    else if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    else if (c >= '0' && c <= '9')
      return c - '0';

    throw std::invalid_argument("[ShapeRenderable] Invalid hex character: " +
                                std::string(1, c));
  };

  // we know there will only ever be 2 hex chars per channel,
  // keep simple impl for now
  int i = 0;
  for (std::string_view hex_str : strs) {
    char h0 = hexToInt(hex_str[0]);
    char h1 = hexToInt(hex_str[1]);
    ret[i++] = static_cast<float>(h0 * 16 + h1) / 255.;
  }

  return ret;
}

void ShapeRenderable::drawGroundPlane() {
  // glDisable(GL_LIGHTING);  // Optional: disable lighting for flat color
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  // glColor3f(0.3f, 0.6f, 0.3f);  // subtle green tone for ground

  glNormal3f(0.0f, 0.0f, 1.0f);  // normal pointing up

  glBegin(GL_QUADS);
  glVertex3f(meta_.x_min, meta_.y_min, 0.0f);
  glVertex3f(meta_.x_max, meta_.y_min, 0.0f);
  glVertex3f(meta_.x_max, meta_.y_max, 0.0f);
  glVertex3f(meta_.x_min, meta_.y_max, 0.0f);
  glEnd();

  // glEnable(GL_LIGHTING);  // Re-enable lighting if needed
}
