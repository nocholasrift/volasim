#include <volasim/event/event.h>
#include <volasim/event/event_dispatcher.h>
#include <volasim/simulation/dynamic_object.h>
#include <volasim/simulation/entity.h>
#include <volasim/simulation/world_buffer.h>

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

[[nodiscard]] const Transform& Entity::getPose(
    const WorldSnapshot& snapshot) const {
  const Transform* pose = snapshot.find(id_);

  return pose != nullptr ? *pose : local_;
}

[[nodiscard]] glm::mat4 Entity::getGlobalTransform(
    const WorldSnapshot& snapshot) const {
  if (parent_) {
    return parent_->getGlobalTransform(snapshot) * getPose(snapshot).toMatrix();
  }

  return getPose(snapshot).toMatrix();
}

void Entity::draw(const WorldSnapshot& snapshot,
                  const glm::mat4& parent_transform, const glm::mat4& view_mat,
                  const glm::mat4& proj_mat, Shader& shader) const {

  const glm::mat4 model_mat = parent_transform * getPose(snapshot).toMatrix();

  // shader will already have been set to used by this point
  if (isVisible() && render_) {
    glm::mat4 mvp = proj_mat * view_mat * model_mat;
    shader.setUniformMat4("mvp", mvp);
    shader.setUniformMat4("model", model_mat);

    render_->draw(shader);
  }

  for (const auto& child : children_) {
    child->draw(snapshot, model_mat, view_mat, proj_mat, shader);
  }
}
