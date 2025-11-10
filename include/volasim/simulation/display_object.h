#ifndef DISPLAYOBJECT_H
#define DISPLAYOBJECT_H

#include <volasim/simulation/shader.h>

#include <volasim/simulation/mesh_renderable.h>
#include <volasim/simulation/shape_renderable.h>

// #include <GL/glu.h>
// #include <GL/glut.h>
#include <SDL3/SDL.h>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

class DisplayObject {

 public:
  DisplayObject();
  DisplayObject(std::string id);
  virtual ~DisplayObject();

  virtual void update();
  virtual void draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
                    Shader& shader);
  virtual void cleanUpDisplayTree();

  virtual void setRenderable(const std::shared_ptr<Renderable> renderable);
  // virtual void setShapeRenderable(const ShapeMetadata& meta);
  // virtual void setMeshRenderable(std::string_view model_fname,
  //                                std::string_view cvx_dcmp_fname);
  //
  void toggleVisibility();
  void makeVisible();
  void makeInvisible();

  void setRotation(const glm::quat& q);
  void setRotation(const glm::vec3& rpy);
  void setTranslation(const glm::vec3& p);

  std::string getID();
  glm::quat getRotation();
  glm::vec3 getTranslation();
  glm::mat4 getLocalTransform();
  glm::mat4 getGlobalTransform();

  const std::shared_ptr<Renderable> getRenderable() const;
  std::shared_ptr<Renderable> getRenderable();

  bool isRenderable();

  DisplayObject* parent_ = NULL;
  bool isCollided_ = false;

  SDL_FlipMode flip_ = SDL_FLIP_NONE;

 protected:
  glm::vec3 position_;
  glm::vec3 scale_;
  glm::quat quaternion_;
  glm::mat4 local_transform_;

  bool is_visible_ = true;
  bool is_renderable_ = false;

  std::string id_;
  std::string type_ = "DisplayObject";

  std::shared_ptr<Renderable> renderable_;
};

#endif
