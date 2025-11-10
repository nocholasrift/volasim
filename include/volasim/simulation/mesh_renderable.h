#ifndef MESHRENDERABLE_H
#define MESHRENDERABLE_H

#include <volasim/simulation/renderable.h>

#include <string>
#include <string_view>

class MeshRenderable : public Renderable {
 public:
  MeshRenderable() {}
  MeshRenderable(std::string_view model_fname,
                 std::string_view cvx_decomp_fname);
  virtual ~MeshRenderable();

  virtual void draw(Shader& shader) override;

  virtual ShapeType getType() const override { return ShapeType::kMesh; }

  virtual void buildFromXML(const pugi::xml_node& item) override;
  void readConvexDecomp();

  const std::vector<Eigen::MatrixX3d>& get_meshes() const {
    return convex_meshes_;
  }

 private:
  std::string_view model_fname_;
  std::string_view cvx_decomp_fname_;

  std::vector<Eigen::MatrixX3d> convex_meshes_;
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;

  unsigned int index_count_ = 0;
};
#endif
