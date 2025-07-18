#include <volasim/simulation/display_object.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

DisplayObject::DisplayObject() {
  id_ = "DEFAULT_ID";

  position_ = glm::vec3(0.f, 0.f, 0.f);
  quaternion_ = glm::quat(0.f, 0.f, 0.f, 1.f);
  scale_ = glm::vec3(1.f, 1.f, 1.f);
}

DisplayObject::DisplayObject(std::string id) {
  id_ = id;

  position_ = glm::vec3(0.f, 0.f, 0.f);
  quaternion_ = glm::quat(0.f, 0.f, 0.f, 1.f);
  scale_ = glm::vec3(1.f, 1.f, 1.f);
}

void DisplayObject::setRenderable(ShapeType type, const ShapeMetadata& meta) {
  renderable_ = std::make_unique<ShapeRenderable>(type, meta);
  is_renderable_ = true;
}

DisplayObject::~DisplayObject() {}

void DisplayObject::update() {}

void DisplayObject::draw() {
  if (!is_visible_)
    return;

  glm::mat4 transform = getLocalTransform();

  if (renderable_) {
    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));
    renderable_->draw();
    glPopMatrix();
  }
}

void DisplayObject::cleanUpDisplayTree() {}

void DisplayObject::toggleVisibility() {
  is_visible_ ^= true;
}

void DisplayObject::makeVisible() {
  is_visible_ = true;
}

void DisplayObject::makeInvisible() {
  is_visible_ = false;
}

void DisplayObject::setRotation(const glm::vec3& rpy) {
  quaternion_ = glm::quat(rpy);
}

void DisplayObject::setRotation(const glm::quat& q) {
  quaternion_ = q;
}

void DisplayObject::setTranslation(const glm::vec3& p) {
  position_ = p;
}

glm::mat4 DisplayObject::getLocalTransform() {
  glm::mat4 trans_mat = glm::translate(glm::mat4(1.0f), position_);
  glm::mat4 rot_mat = glm::mat4_cast(quaternion_);
  glm::mat4 scale_mat = glm::scale(glm::mat4(1.0f), scale_);

  local_transform_ = trans_mat * rot_mat * scale_mat;
  return local_transform_;
}

glm::mat4 DisplayObject::getGlobalTransform() {
  if (parent_)
    return parent_->getGlobalTransform() * getLocalTransform();
  else
    return getLocalTransform();
}

glm::vec3 DisplayObject::getPosition() {
  return position_;
}

glm::quat DisplayObject::getRotation() {
  return quaternion_;
}

const Renderable& DisplayObject::getRenderable(){
  return *renderable_;
}

std::string DisplayObject::getID() {
  return id_;
}

bool DisplayObject::isRenderable(){
  return is_renderable_;
}
