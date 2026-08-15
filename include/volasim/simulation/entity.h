#ifndef ENTITY_H
#define ENTITY_H

#include <volasim/simulation/renderable.h>
#include <volasim/simulation/shader.h>
#include <volasim/simulation/transform.h>

#include <atomic>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

class DynamicObject;
class WorldSnapshot;

using EntityID = std::uint32_t;

class EntityFactory;

class Entity {
 public:
  // passkey pattern to only allow construction in factory
  class ConstructorToken {
    friend class EntityFactory;

   private:
    ConstructorToken() = default;
  };

  Entity(ConstructorToken, EntityID id, std::string_view name);

  // Must define because DynamicObject is forward declared and Entity has a member
  // unique_ptr<DynamicObject>. If not declared here cpp would inline a definition
  // attempting to delete the member (on an incomplete forward declared type),
  // causing an error. Declaring entity here and defining empty destructor in cpp
  // avoids this.
  ~Entity();

  Entity& addChild(std::unique_ptr<Entity> child);
  void    removeChild(EntityID target_id);

  void setTranslation(const glm::vec3& trans) { local_.position = trans; }
  void setRotation(const glm::quat& quat) { local_.rotation = quat; }
  void setRotation(const glm::vec3& rpy) { local_.rotation = glm::quat(rpy); }

  [[nodiscard]] glm::vec3 getTranslation() const { return local_.position; }
  [[nodiscard]] glm::quat getRotation() const { return local_.rotation; }
  [[nodiscard]] const Transform& getLocalTransform() const { return local_; }

  // Pose to display this entity with: its physics pose when the snapshot has
  // one, otherwise the transform it was authored with.
  [[nodiscard]] const Transform& getPose(const WorldSnapshot& snapshot) const;
  [[nodiscard]] glm::mat4        getGlobalTransform(
             const WorldSnapshot& snapshot) const;

  [[nodiscard]] bool isVisible() const { return is_visible_; }

  // renderable
  void setRenderable(std::shared_ptr<Renderable> r) { render_ = std::move(r); }
  [[nodiscard]] bool isRenderable() const { return render_ != nullptr; }
  [[nodiscard]] const std::shared_ptr<Renderable>& getRenderable() const {
    return render_;
  }

  // dynamics
  [[nodiscard]] bool isDynamic() const { return dynamic_object_ != nullptr; }
  [[nodiscard]] DynamicObject& getDynamics() { return *dynamic_object_; }
  void setDynamics(std::unique_ptr<DynamicObject> dynamics);

  // parent_transform is the global transform of this node's parent; the root is
  // drawn with identity. Accumulating downwards keeps the traversal linear in
  // the number of nodes instead of walking back up to the root at every node.
  void draw(const WorldSnapshot& snapshot, const glm::mat4& parent_transform,
            const glm::mat4& view, const glm::mat4& proj, Shader& shader) const;

  [[nodiscard]] EntityID getID() const { return id_; }

 private:
  // Dispatch OBJ_RM for this node and its whole subtree (children first), so
  // physics destroys every associated body before the nodes are freed.
  void notifyRemoval();

  EntityID                           id_;
  Transform                          local_;
  std::string                        name_;
  Entity*                            parent_{nullptr};
  std::list<std::unique_ptr<Entity>> children_;
  std::shared_ptr<Renderable>        render_;
  std::unique_ptr<DynamicObject>     dynamic_object_;
  bool                               is_visible_{true};
};

class EntityFactory {
 public:
  std::unique_ptr<Entity> create(std::string_view name) {
    std::uint32_t id = next_id.fetch_add(1, std::memory_order_relaxed);

    return std::make_unique<Entity>(Entity::ConstructorToken{}, id, name);
  }

 private:
  std::atomic<std::uint32_t> next_id{1};
};

#endif
