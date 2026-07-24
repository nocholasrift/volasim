#include <volasim/event/event.h>
#include <volasim/event/event_dispatcher.h>
#include <volasim/simulation/dynamic_object.h>
#include <volasim/simulation/entity.h>

Entity::Entity(ConstructorToken /*unused*/, EntityID id, std::string_view name)
    : id_(id), name_(name) {}

Entity::~Entity() = default;

void Entity::setDynamics(std::unique_ptr<DynamicObject> dynamics) {
  dynamic_object_ = std::move(dynamics);
}

Entity& Entity::addChild(std::unique_ptr<Entity> child) {
  child->parent_ = this;
  Entity* raw    = child.get();
  children_.push_back(std::move(child));

  DisplayEvent event("OBJ_ADD", &EventDispatcher::getInstance(), raw);
  EventDispatcher::getInstance().dispatchEvent(&event);

  return *children_.back();
}

void Entity::removeChild(EntityID target_id) {
  for (auto it = children_.begin(); it != children_.end(); ++it) {
    if ((*it)->getID() == target_id) {
      (*it)->notifyRemoval();
      children_.erase(it);
      return;
    }
  }
}

void Entity::notifyRemoval() {
  for (const auto& child : children_) {
    child->notifyRemoval();
  }

  DisplayEvent event("OBJ_RM", &EventDispatcher::getInstance(), this);
  EventDispatcher::getInstance().dispatchEvent(&event);
}

[[nodiscard]] glm::mat4 Entity::getLocalTransform() const {
  glm::mat4 trans_mat = glm::translate(glm::mat4(1.0F), local_.position);
  glm::mat4 rot_mat   = glm::mat4_cast(local_.rotation);
  glm::mat4 scale_mat = glm::scale(glm::mat4(1.0F), local_.scale);

  return trans_mat * rot_mat * scale_mat;
}

[[nodiscard]] glm::mat4 Entity::getGlobalTransform() const {
  if (parent_) {
    return parent_->getGlobalTransform() * getLocalTransform();
  }

  return getLocalTransform();
}

void Entity::draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
                  Shader& shader) {

  // shader will already have been set to used by this point
  if (isVisible() && render_) {
    glm::mat4 model_mat = getGlobalTransform();
    glm::mat4 mvp       = proj_mat * view_mat * model_mat;
    shader.setUniformMat4("mvp", mvp);
    shader.setUniformMat4("model", model_mat);

    render_->draw(shader);
  }

  for (const auto& child : children_) {
    child->draw(view_mat, proj_mat, shader);
  }
}
