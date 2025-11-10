#include <glad/glad.h>

#include <volasim/simulation/mesh_renderable.h>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <array>
#include <iostream>
#include <stdexcept>

MeshRenderable::MeshRenderable(std::string_view model_fname,
                               std::string_view cvx_decomp_fname)
    : model_fname_(model_fname), cvx_decomp_fname_(cvx_decomp_fname) {
  // createBuffer();
  // readConvexDecomp();
}

MeshRenderable::~MeshRenderable() {}

void MeshRenderable::draw(Shader& shader) {
  shader.setUniformVec3("color", glm::vec3(.6, .6, .6));
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, (void*)0);
  glBindVertexArray(0);
}

void MeshRenderable::readConvexDecomp() {
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
      std::string(cvx_decomp_fname_).c_str(),
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
          aiProcess_PreTransformVertices);

  if (!scene || !scene->HasMeshes()) {
    throw std::runtime_error("Failed to load mesh: " +
                             std::string(cvx_decomp_fname_));
  }

  convex_meshes_.clear();
  convex_meshes_.reserve(scene->mNumMeshes);
  for (size_t i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh* model = scene->mMeshes[i];
    Eigen::MatrixX3d& verts =
        convex_meshes_.emplace_back(model->mNumVertices, 3);
    // std::cout << i << " -- verts: " << model->mNumVertices << "\n";
    for (size_t j = 0; j < model->mNumVertices; ++j) {
      aiVector3D aiv = model->mVertices[j];
      verts.row(j) = Eigen::Vector3d(aiv.x, aiv.y, aiv.z);
    }
  }
}

void MeshRenderable::buildFromXML(const pugi::xml_node& item) {
  pugi::xml_node geometry_node = item.child("geometry");

  model_fname_ = geometry_node.attribute("model_file").as_string();
  cvx_decomp_fname_ = geometry_node.attribute("decomp_file").as_string();

  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
      std::string(model_fname_).c_str(),
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
          aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace |
          aiProcess_PreTransformVertices);

  if (!scene || !scene->HasMeshes()) {
    throw std::runtime_error("Failed to load mesh: " +
                             std::string(model_fname_));
  }

  const aiMesh* model = scene->mMeshes[0];
  std::vector<float> vertices;
  std::vector<unsigned int> indices;

  const aiVector3D aiVectorZero(0.f, 0.f, 0.f);
  for (size_t i = 0; i < model->mNumVertices; ++i) {
    const aiVector3D& pos = model->mVertices[i];
    const aiVector3D& normal =
        model->HasNormals() ? model->mNormals[i] : aiVector3D(0.f, 0.f, 1.f);
    const aiVector3D& uv = model->HasTextureCoords(0)
                               ? model->mTextureCoords[0][i]
                               : aiVector3D(0.f, 0.f, 0.f);

    vertices.insert(vertices.end(), {pos.x, pos.y, pos.z, normal.x, normal.y,
                                     normal.z, uv.x, uv.y});
  }

  // Extract indices
  for (unsigned int f = 0; f < model->mNumFaces; ++f) {
    const aiFace& face = model->mFaces[f];
    if (face.mNumIndices != 3) {
      std::cerr << "[Mesh Renderable] Warning: Mesh did not "
                   "triangularize\n";
      continue;  // should be triangulated
    }
    indices.push_back(face.mIndices[0]);
    indices.push_back(face.mIndices[1]);
    indices.push_back(face.mIndices[2]);
  }

  // buffer setup
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
               vertices.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
               indices.data(), GL_STATIC_DRAW);

  // Layout: pos(3), normal(3), uv(2)
  GLsizei stride = 8 * sizeof(float);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                        (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride,
                        (void*)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);

  glBindVertexArray(0);

  index_count_ = indices.size();

  readConvexDecomp();
}
