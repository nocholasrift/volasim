#include <volasim/simulation/display_object.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

DisplayObject::DisplayObject() {
  id_ = "DEFAULT_ID";

  position_ = glm::vec3(0.f, 0.f, 0.f);
  quaternion_ = glm::quat(1.f, 0.f, 0.f, 0.f);
  scale_ = glm::vec3(1.f, 1.f, 1.f);
}

DisplayObject::DisplayObject(std::string id) {
  id_ = id;

  position_ = glm::vec3(0.f, 0.f, 0.f);
  quaternion_ = glm::quat(1.f, 0.f, 0.f, 0.f);
  scale_ = glm::vec3(1.f, 1.f, 1.f);
}

void DisplayObject::setRenderable(
    const std::shared_ptr<Renderable> renderable) {
  renderable_ = renderable;
  is_renderable_ = true;
}

// void DisplayObject::setShapeRenderable(const ShapeMetadata& meta) {
//   renderable_ = std::make_unique<ShapeRenderable>(meta);
//   is_renderable_ = true;
// }
//
// void DisplayObject::setMeshRenderable(std::string_view model_fname,
//                                       std::string_view cvx_dcmp_fname) {
//   renderable_ = std::make_unique<MeshRenderable>(model_fname, cvx_dcmp_fname);
//   is_renderable_ = true;
// }

DisplayObject::~DisplayObject() {}

void DisplayObject::update() {}

void DisplayObject::draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
                         Shader& shader) {
  if (!is_visible_)
    return;

  // shader will already have been set to used by this point
  if (renderable_) {
    glm::mat4 model_mat;
    if (parent_)
      model_mat = parent_->getGlobalTransform() * getLocalTransform();
    else
      model_mat = getLocalTransform();

    glm::mat4 mvp = proj_mat * view_mat * model_mat;
    shader.setUniformMat4("mvp", mvp);
    shader.setUniformMat4("model", model_mat);

    renderable_->draw(shader);
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

glm::vec3 DisplayObject::getTranslation() {
  return position_;
}

glm::quat DisplayObject::getRotation() {
  return quaternion_;
}

const std::shared_ptr<Renderable> DisplayObject::getRenderable() const {
  return renderable_;
}

std::shared_ptr<Renderable> DisplayObject::getRenderable() {
  return renderable_;
}

std::string DisplayObject::getID() {
  return id_;
}

bool DisplayObject::isRenderable() {
  return is_renderable_;
}
