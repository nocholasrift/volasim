#ifndef DISPLAYOBJECT_H
#define DISPLAYOBJECT_H

#include <GL/glu.h>
#include <GL/glut.h>
#include <SDL3/SDL.h>
#include <volasim/simulation/shape_renderable.h>
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
  virtual void draw();
  virtual void cleanUpDisplayTree();

  void toggleVisibility();
  void makeVisible();
  void makeInvisible();

  void setRotation(const glm::vec3& rpy);
  void setTranslation(const glm::vec3& p);
  void setRenderable(ShapeType type, const ShapeMetadata& meta);

  std::string getID();
  glm::quat getRotation();
  SDL_Point getPivot();
  SDL_Point getPosition();
  SDL_Point getGlobalHitbox();
  glm::mat4 getLocalTransform();
  glm::mat4 getGlobalTransform();

  void setHitbox(const std::vector<SDL_Point>& points);
  void setHitbox(double boundLow, double boundHigh);
  void setHitbox(double boundLowX, double boundHighX, double boundLowY,
                 double boundHighY);

  DisplayObject* parent = NULL;
  bool isCollided = false;

  SDL_FlipMode flip_ = SDL_FLIP_NONE;

 protected:
  /* Texture currently being drawn. Equal to texture for normal DO */
  SDL_Texture* curr_texture_;

  SDL_Texture* texture_ = NULL;
  SDL_Surface* image_ = NULL;

  std::vector<SDL_Point> global_hitbox_;
  std::vector<SDL_Point> hitbox_;

  glm::vec3 position_;
  glm::vec3 scale_;
  glm::quat quaternion_;
  glm::mat4 local_transform_;

  int alpha_ = 255;
  bool show_hitbox_ = false;
  bool is_visible_ = true;

  std::string id_;
  std::string type_ = "DisplayObject";

  std::unique_ptr<Renderable> renderable_;
};

#endif
