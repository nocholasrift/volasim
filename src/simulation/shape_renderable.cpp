#include <glad/glad.h>

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

void ShapeRenderable::buildFromXML(const pugi::xml_node& item) {

  pugi::xml_node geometry_node = item.child("geometry");
  meta_.type = shape_map_[geometry_node.attribute("type").as_string()];

  meta_.color = item.child_value("color");

  if (meta_.color.length() != 7) {
    std::string err_str =
        "[XMLParser] Invalid color string! " + meta_.color +
        "\nShould be formatted as 6 hex digits preceeded by #";
    throw std::invalid_argument(err_str);
  }

  if (meta_.color[0] != '#') {
    throw std::invalid_argument("[XMLParser] Color must start with #");
  }

  for (size_t i = 1; i < meta_.color.length(); ++i) {
    if (!std::isxdigit(meta_.color[i])) {
      throw std::invalid_argument("[XMLParser] Invalid hex digit in color: " +
                                  meta_.color);
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
      float n_sectors = 32;

      meta_.radius = std::stof(geometry_node.attribute("radius").as_string());
      meta_.height = std::stof(geometry_node.attribute("length").as_string());

      // Cylinder cylinder(meta_.radius, meta_.radius, meta_.height, 32,
      //                   2);
      std::vector<float> vertices;
      std::vector<float> normals;
      for (int i = 0; i < 2; ++i) {
        float h = i * meta_.height;

        float sector_angle;
        for (int j = 0; j <= n_sectors; ++j) {
          sector_angle = j * 2 * M_PI / n_sectors;
          float vx = meta_.radius * cos(sector_angle);
          float vy = meta_.radius * sin(sector_angle);

          vertices.push_back(vx);
          vertices.push_back(vy);
          vertices.push_back(h);

          normals.push_back(vx / meta_.radius);
          normals.push_back(vy / meta_.radius);
          normals.push_back(0.);
        }
      }

      int base_center_idx = (int)vertices.size() / 3;
      int top_center_idx = base_center_idx + n_sectors + 1;

      for (int i = 0; i < 2; ++i) {
        float h = i * meta_.height;
        float nz = 2 * i - 1;

        vertices.push_back(0);
        vertices.push_back(0);
        vertices.push_back(h);

        normals.push_back(0);
        normals.push_back(0);
        normals.push_back(nz);

        float sector_angle;
        for (int j = 0; j <= n_sectors; ++j) {
          sector_angle = j * 2 * M_PI / n_sectors;

          float vx = meta_.radius * cos(sector_angle);
          float vy = meta_.radius * sin(sector_angle);

          vertices.push_back(vx);
          vertices.push_back(vy);
          vertices.push_back(h);

          normals.push_back(0);
          normals.push_back(0);
          normals.push_back(nz);
        }
      }

      // merge indices and normals together
      std::vector<float> vert_norms;
      for (size_t i = 0; i < vertices.size(); i += 3) {
        vert_norms.push_back(vertices[i]);
        vert_norms.push_back(vertices[i + 1]);
        vert_norms.push_back(vertices[i + 2]);

        vert_norms.push_back(normals[i]);
        vert_norms.push_back(normals[i + 1]);
        vert_norms.push_back(normals[i + 2]);
      }

      // int base_center_idx = 2 * (n_sectors + 1);
      // int top_center_idx = base_center_idx + n_sectors + 1;

      std::vector<int> indices;
      int k1 = 0;
      int k2 = n_sectors + 1;

      // indices for side surface
      for (int i = 0; i < n_sectors; ++i, ++k1, ++k2) {
        // tri 1
        indices.push_back(k1);
        indices.push_back(k1 + 1);
        indices.push_back(k2);

        // tri 2
        indices.push_back(k2);
        indices.push_back(k1 + 1);
        indices.push_back(k2 + 1);
      }

      // indices for base
      for (int i = 0, k = base_center_idx + 1; i < n_sectors; ++i, ++k) {
        if (i < n_sectors - 1) {
          indices.push_back(base_center_idx);
          indices.push_back(k + 1);
          indices.push_back(k);
        } else {
          indices.push_back(base_center_idx);
          indices.push_back(base_center_idx + 1);
          indices.push_back(k);
        }
      }

      // indices for top
      for (int i = 0, k = top_center_idx + 1; i < n_sectors; ++i, ++k) {
        if (i < n_sectors - 1) {
          indices.push_back(top_center_idx);
          indices.push_back(k);
          indices.push_back(k + 1);
        } else {
          indices.push_back(top_center_idx);
          indices.push_back(k);
          indices.push_back(top_center_idx + 1);
        }
      }

      // glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
      //              vertices.data(), GL_STATIC_DRAW);
      glBufferData(GL_ARRAY_BUFFER, vert_norms.size() * sizeof(float),
                   vert_norms.data(), GL_STATIC_DRAW);

      // copy index data to VBO
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int),
                   indices.data(), GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);

      // normals
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      meta_.index_count = indices.size();

      break;
    }  // end case kCylinder
    case ShapeType::kPlane: {
      meta_.x_min = std::stof(geometry_node.attribute("x_min").as_string());
      meta_.x_max = std::stof(geometry_node.attribute("x_max").as_string());
      meta_.y_min = std::stof(geometry_node.attribute("y_min").as_string());
      meta_.y_max = std::stof(geometry_node.attribute("y_max").as_string());
      meta_.z = std::stof(geometry_node.attribute("z").as_string());
      meta_.name = item.attribute("class").as_string();

      float ground_verts[] = {
          //positions
          meta_.x_max, meta_.y_max, meta_.z, 0, 0, 1,
          meta_.x_max, meta_.y_min, meta_.z, 0, 0, 1,
          meta_.x_min, meta_.y_min, meta_.z, 0, 0, 1,
          meta_.x_min, meta_.y_max, meta_.z, 0, 0, 1,
      };

      unsigned int indices[] = {0, 1, 3, 1, 2, 3};

      // float normals[] = {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};

      glBufferData(GL_ARRAY_BUFFER, sizeof(ground_verts), ground_verts,
                   GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);

      // normals
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                   GL_STATIC_DRAW);
      meta_.index_count = 6;

      break;
    }  // end case kPlane
    case ShapeType::kCube: {
      // meta_.x_mi
      meta_.size = std::stof(geometry_node.attribute("size").as_string());
      float sz = meta_.size;

      float verts[] = {
          // left face
          -sz / 2, -sz / 2, -sz / 2, -1, 0, 0, -sz / 2, sz / 2, -sz / 2, -1, 0,
          0, -sz / 2, sz / 2, sz / 2, -1, 0, 0, -sz / 2, -sz / 2, -sz / 2, -1,
          0, 0, -sz / 2, sz / 2, sz / 2, -1, 0, 0, -sz / 2, -sz / 2, sz / 2, -1,
          0, 0,

          // back face
          -sz / 2, -sz / 2, -sz / 2, 0, -1, 0, -sz / 2, -sz / 2, sz / 2, 0, -1,
          0, sz / 2, -sz / 2, -sz / 2, 0, -1, 0, -sz / 2, -sz / 2, sz / 2, 0,
          -1, 0, sz / 2, -sz / 2, sz / 2, 0, -1, 0, sz / 2, -sz / 2, -sz / 2, 0,
          -1, 0,

          // right face
          sz / 2, -sz / 2, sz / 2, 1, 0, 0, sz / 2, sz / 2, sz / 2, 1, 0, 0,
          sz / 2, -sz / 2, -sz / 2, 1, 0, 0, sz / 2, sz / 2, sz / 2, 1, 0, 0,
          sz / 2, sz / 2, -sz / 2, 1, 0, 0, sz / 2, -sz / 2, -sz / 2, 1, 0, 0,

          // front face
          -sz / 2, sz / 2, -sz / 2, 0, 1, 0, -sz / 2, sz / 2, sz / 2, 0, 1, 0,
          sz / 2, sz / 2, -sz / 2, 0, 1, 0, -sz / 2, sz / 2, sz / 2, 0, 1, 0,
          sz / 2, sz / 2, sz / 2, 0, 1, 0, sz / 2, sz / 2, -sz / 2, 0, 1, 0,

          // top face
          -sz / 2, sz / 2, sz / 2, 0, 0, 1, -sz / 2, -sz / 2, sz / 2, 0, 0, 1,
          sz / 2, -sz / 2, sz / 2, 0, 0, 1, -sz / 2, sz / 2, sz / 2, 0, 0, 1,
          sz / 2, -sz / 2, sz / 2, 0, 0, 1, sz / 2, sz / 2, sz / 2, 0, 0, 1,

          // bottom face
          -sz / 2, -sz / 2, -sz / 2, 0, 0, -1, -sz / 2, sz / 2, -sz / 2, 0, 0,
          -1, sz / 2, sz / 2, -sz / 2, 0, 0, -1, -sz / 2, -sz / 2, -sz / 2, 0,
          0, -1, sz / 2, sz / 2, -sz / 2, 0, 0, -1, sz / 2, -sz / 2, -sz / 2, 0,
          0, -1

      };

      int indices[36];
      for (int i = 0; i < 36; ++i)
        indices[i] = i;

      // float normals[] = {-1, 0, 0, -1, 0, 0, 0, -1, 0,  0, -1, 0,
      //                    1,  0, 0, 1,  0, 0, 0, 1,  0,  0, 1,  0,
      //                    0,  0, 1, 0,  0, 1, 0, 0,  -1, 0, 0,  -1};

      glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)0);

      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                            (void*)(3 * sizeof(float)));

      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                   GL_STATIC_DRAW);

      meta_.index_count = 36;

      break;
    }
    default:
      break;
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
