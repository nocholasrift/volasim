#ifndef VOLASIM_MESH_PRIMITIVES_H
#define VOLASIM_MESH_PRIMITIVES_H

#include <glm/gtc/constants.hpp>

#include <cmath>
#include <vector>

namespace volasim::primitives {

struct MeshData {
  std::vector<float>        vertices;  // interleaved [pos3, normal3]
  std::vector<unsigned int> indices;
};

// Unit cylinder: radius=1, height=1, base at z=0, no caps.
inline MeshData cylinder(int sectors = 32) {
  MeshData mesh;

  const float two_pi = 2.f * glm::pi<float>();

  // Side vertices: two rings (z=0 and z=1).
  for (int ring = 0; ring < 2; ++ring) {
    float z = static_cast<float>(ring);
    for (int j = 0; j <= sectors; ++j) {
      float angle = j * two_pi / sectors;
      float cx    = std::cos(angle);
      float cy    = std::sin(angle);

      mesh.vertices.push_back(cx);   // pos.x
      mesh.vertices.push_back(cy);   // pos.y
      mesh.vertices.push_back(z);    // pos.z
      mesh.vertices.push_back(cx);   // normal.x
      mesh.vertices.push_back(cy);   // normal.y
      mesh.vertices.push_back(0.f);  // normal.z
    }
  }

  // Side indices.
  int k1 = 0;
  int k2 = sectors + 1;
  for (int i = 0; i < sectors; ++i, ++k1, ++k2) {
    mesh.indices.push_back(k1);
    mesh.indices.push_back(k1 + 1);
    mesh.indices.push_back(k2);

    mesh.indices.push_back(k2);
    mesh.indices.push_back(k1 + 1);
    mesh.indices.push_back(k2 + 1);
  }

  return mesh;
}

// Cylinder with caps: given radius and height, base at z=0.
inline MeshData cappedCylinder(float radius, float height, int sectors = 32) {
  MeshData mesh;

  const float two_pi = 2.f * glm::pi<float>();

  // Side vertices: two rings.
  for (int ring = 0; ring < 2; ++ring) {
    float z = ring * height;
    for (int j = 0; j <= sectors; ++j) {
      float angle = j * two_pi / sectors;
      float cx    = radius * std::cos(angle);
      float cy    = radius * std::sin(angle);

      mesh.vertices.push_back(cx);
      mesh.vertices.push_back(cy);
      mesh.vertices.push_back(z);
      mesh.vertices.push_back(cx / radius);
      mesh.vertices.push_back(cy / radius);
      mesh.vertices.push_back(0.f);
    }
  }

  int base_center_idx = static_cast<int>(mesh.vertices.size() / 6);
  int top_center_idx  = base_center_idx + sectors + 2;

  // Cap vertices: center + ring, with flat normals.
  for (int cap = 0; cap < 2; ++cap) {
    float z  = cap * height;
    float nz = 2.f * cap - 1.f;

    // Center vertex.
    mesh.vertices.push_back(0.f);
    mesh.vertices.push_back(0.f);
    mesh.vertices.push_back(z);
    mesh.vertices.push_back(0.f);
    mesh.vertices.push_back(0.f);
    mesh.vertices.push_back(nz);

    for (int j = 0; j <= sectors; ++j) {
      float angle = j * two_pi / sectors;
      float cx    = radius * std::cos(angle);
      float cy    = radius * std::sin(angle);

      mesh.vertices.push_back(cx);
      mesh.vertices.push_back(cy);
      mesh.vertices.push_back(z);
      mesh.vertices.push_back(0.f);
      mesh.vertices.push_back(0.f);
      mesh.vertices.push_back(nz);
    }
  }

  // Side indices.
  int k1 = 0;
  int k2 = sectors + 1;
  for (int i = 0; i < sectors; ++i, ++k1, ++k2) {
    mesh.indices.push_back(k1);
    mesh.indices.push_back(k1 + 1);
    mesh.indices.push_back(k2);

    mesh.indices.push_back(k2);
    mesh.indices.push_back(k1 + 1);
    mesh.indices.push_back(k2 + 1);
  }

  // Base cap indices.
  for (int i = 0, k = base_center_idx + 1; i < sectors; ++i, ++k) {
    if (i < sectors - 1) {
      mesh.indices.push_back(base_center_idx);
      mesh.indices.push_back(k + 1);
      mesh.indices.push_back(k);
    } else {
      mesh.indices.push_back(base_center_idx);
      mesh.indices.push_back(base_center_idx + 1);
      mesh.indices.push_back(k);
    }
  }

  // Top cap indices.
  for (int i = 0, k = top_center_idx + 1; i < sectors; ++i, ++k) {
    if (i < sectors - 1) {
      mesh.indices.push_back(top_center_idx);
      mesh.indices.push_back(k);
      mesh.indices.push_back(k + 1);
    } else {
      mesh.indices.push_back(top_center_idx);
      mesh.indices.push_back(k);
      mesh.indices.push_back(top_center_idx + 1);
    }
  }

  return mesh;
}

inline MeshData cube(float size) {
  float s = size / 2.f;

  MeshData mesh;
  // clang-format off
  mesh.vertices = {
    // left face (-X)
    -s, -s, -s, -1, 0, 0,  -s,  s, -s, -1, 0, 0,  -s,  s,  s, -1, 0, 0,
    -s, -s, -s, -1, 0, 0,  -s,  s,  s, -1, 0, 0,  -s, -s,  s, -1, 0, 0,
    // back face (-Y)
    -s, -s, -s,  0, -1, 0, -s, -s,  s,  0, -1, 0,  s, -s, -s,  0, -1, 0,
    -s, -s,  s,  0, -1, 0,  s, -s,  s,  0, -1, 0,  s, -s, -s,  0, -1, 0,
    // right face (+X)
     s, -s,  s,  1, 0, 0,   s,  s,  s,  1, 0, 0,   s, -s, -s,  1, 0, 0,
     s,  s,  s,  1, 0, 0,   s,  s, -s,  1, 0, 0,   s, -s, -s,  1, 0, 0,
    // front face (+Y)
    -s,  s, -s,  0, 1, 0,  -s,  s,  s,  0, 1, 0,   s,  s, -s,  0, 1, 0,
    -s,  s,  s,  0, 1, 0,   s,  s,  s,  0, 1, 0,   s,  s, -s,  0, 1, 0,
    // top face (+Z)
    -s,  s,  s,  0, 0, 1,  -s, -s,  s,  0, 0, 1,   s, -s,  s,  0, 0, 1,
    -s,  s,  s,  0, 0, 1,   s, -s,  s,  0, 0, 1,   s,  s,  s,  0, 0, 1,
    // bottom face (-Z)
    -s, -s, -s,  0, 0, -1, -s,  s, -s,  0, 0, -1,  s,  s, -s,  0, 0, -1,
    -s, -s, -s,  0, 0, -1,  s,  s, -s,  0, 0, -1,  s, -s, -s,  0, 0, -1,
  };
  // clang-format on

  mesh.indices.resize(36);
  for (int i = 0; i < 36; ++i) {
    mesh.indices[i] = i;
  }

  return mesh;
}

inline MeshData plane(float x_min, float x_max, float y_min, float y_max,
                      float z) {
  MeshData mesh;
  // clang-format off
  mesh.vertices = {
    x_max, y_max, z, 0, 0, 1,
    x_max, y_min, z, 0, 0, 1,
    x_min, y_min, z, 0, 0, 1,
    x_min, y_max, z, 0, 0, 1,
  };
  mesh.indices = {0, 1, 3, 1, 2, 3};
  // clang-format on

  return mesh;
}

}  // namespace volasim::primitives

#endif
