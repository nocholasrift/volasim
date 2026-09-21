#include <glad/glad.h>

#include <volasim/simulation/mesh_primitives.h>
#include <volasim/simulation/shape_renderable.h>

#include <array>
#include <iostream>
#include <stdexcept>

ShapeRenderable::ShapeRenderable(const ShapeMetadata& meta) : meta_(meta) {}

ShapeRenderable::~ShapeRenderable() {}

void ShapeRenderable::draw(Shader& shader) {

  shader.setUniformVec3("color", hexToRGB(meta_.color));
  glBindVertexArray(meta_.vao);
  glDrawElements(GL_TRIANGLES, meta_.index_count, GL_UNSIGNED_INT, (void*)0);
  glBindVertexArray(0);
}

glm::vec3 ShapeRenderable::hexToRGB(std::string_view hex_str) {
  if (hex_str[0] != '#' || hex_str.length() != 7) {
    std::string err_str = "[ShapeRenderable] Invalid color string! " +
                          std::string(hex_str) +
                          "\nShould be formatted as 6 hex digits preceded by #";
    throw std::invalid_argument(err_str);
  }

  glm::vec3        ret;
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
    char h0  = hexToInt(hex_str[0]);
    char h1  = hexToInt(hex_str[1]);
    ret[i++] = static_cast<float>(h0 * 16 + h1) / 255.;
  }

  return ret;
}

void ShapeRenderable::buildFromXML(const pugi::xml_node& item) {

  pugi::xml_node geometry_node = item.child("geometry");
  meta_.type = shape_map_[geometry_node.attribute("type").as_string()];

  meta_.color = item.child_value("color");

  if (meta_.color.length() != 7) {
    std::string err_str =
        "Invalid color string: '" + meta_.color +
        "'\nShould be formatted as 6 hex digits preceded by #";
    throw std::invalid_argument(err_str);
  }

  if (meta_.color[0] != '#') {
    throw std::invalid_argument("Color must start with #");
  }

  for (size_t i = 1; i < meta_.color.length(); ++i) {
    if (!std::isxdigit(static_cast<unsigned char>(meta_.color[i]))) {
      throw std::invalid_argument("Invalid hex digit in color: " + meta_.color);
    }
  }

  glGenVertexArrays(1, &meta_.vao);
  glBindVertexArray(meta_.vao);

  glGenBuffers(1, &meta_.vbo);
  glBindBuffer(GL_ARRAY_BUFFER, meta_.vbo);

  glGenBuffers(1, &meta_.ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meta_.ebo);

  switch (meta_.type) {
    case ShapeType::kSphere:
      break;
    case ShapeType::kCylinder: {
      meta_.radius = std::stof(geometry_node.attribute("radius").as_string());
      meta_.height = std::stof(geometry_node.attribute("length").as_string());

      auto mesh =
          volasim::primitives::cappedCylinder(meta_.radius, meta_.height);

      glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float),
                   mesh.vertices.data(), GL_STATIC_DRAW);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   mesh.indices.size() * sizeof(unsigned int),
                   mesh.indices.data(), GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      meta_.index_count = static_cast<GLsizei>(mesh.indices.size());

      break;
    }  // end case kCylinder
    case ShapeType::kPlane: {
      meta_.x_min = std::stof(geometry_node.attribute("x_min").as_string());
      meta_.x_max = std::stof(geometry_node.attribute("x_max").as_string());
      meta_.y_min = std::stof(geometry_node.attribute("y_min").as_string());
      meta_.y_max = std::stof(geometry_node.attribute("y_max").as_string());
      meta_.z     = std::stof(geometry_node.attribute("z").as_string());
      meta_.name  = item.attribute("class").as_string();

      auto mesh = volasim::primitives::plane(meta_.x_min, meta_.x_max,
                                             meta_.y_min, meta_.y_max, meta_.z);

      glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float),
                   mesh.vertices.data(), GL_STATIC_DRAW);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   mesh.indices.size() * sizeof(unsigned int),
                   mesh.indices.data(), GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      meta_.index_count = static_cast<GLsizei>(mesh.indices.size());

      break;
    }  // end case kPlane
    case ShapeType::kCube: {
      meta_.size = std::stof(geometry_node.attribute("size").as_string());

      auto mesh = volasim::primitives::cube(meta_.size);

      glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float),
                   mesh.vertices.data(), GL_STATIC_DRAW);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   mesh.indices.size() * sizeof(unsigned int),
                   mesh.indices.data(), GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      meta_.index_count = static_cast<GLsizei>(mesh.indices.size());

      break;
    }
    default:
      break;
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
