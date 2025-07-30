#include <glm/gtc/type_ptr.hpp>

#include <volasim/event/event.h>
#include <volasim/event/event_dispatcher.h>
#include <volasim/simulation/display_object_container.h>

DisplayObjectContainer::DisplayObjectContainer() : DisplayObject() {}

DisplayObjectContainer::DisplayObjectContainer(std::string id)
    : DisplayObject(id) {}

DisplayObjectContainer::~DisplayObjectContainer() {
  removeAllChildren();
}

void DisplayObjectContainer::addChild(DisplayObject* child) {
  // Simulation::event_handler_.addEventLis
  children.push_back(child);
  child->parent_ = this;

  DisplayEvent e("OBJ_ADD", &EventDispatcher::getInstance(), child);
  EventDispatcher::getInstance().dispatchEvent(&e);
}

void DisplayObjectContainer::removeChild(DisplayObject* child,
                                         bool should_delete) {

  for (std::vector<DisplayObject*>::iterator it = children.begin();
       it != children.end(); it++) {
    if (*it == child) {
      if (should_delete) {
        delete *it;
        *it = nullptr;
      }
      children.erase(it);

      DisplayEvent e("OBJ_RM", &EventDispatcher::getInstance(), child);
      EventDispatcher::getInstance().dispatchEvent(&e);
      return;
    }
  }
}

void DisplayObjectContainer::removeChild(std::string id, bool should_delete) {

  for (std::vector<DisplayObject*>::iterator it = children.begin();
       it != children.end(); it++) {
    if ((*it)->getID() == id) {
      DisplayEvent e("OBJ_RM", &EventDispatcher::getInstance(), *it);
      EventDispatcher::getInstance().dispatchEvent(&e);

      if (should_delete) {
        delete *it;
        *it = nullptr;
      }
      children.erase(it);

      return;
    }
  }
}

void DisplayObjectContainer::removeChild(int index, bool should_delete) {
  if (index >= children.size())
    return;

  std::vector<DisplayObject*>::iterator it = children.begin() + index;

  DisplayEvent e("OBJ_RM", &EventDispatcher::getInstance(), *it);
  EventDispatcher::getInstance().dispatchEvent(&e);

  if (should_delete) {
    delete *it;
    *it = nullptr;
  }
  children.erase(it);
}

void DisplayObjectContainer::removeAllChildren() {
  for (std::vector<DisplayObject*>::iterator it = children.begin();
       it != children.end(); it++) {

    DisplayEvent e("OBJ_RM", &EventDispatcher::getInstance(), *it);
    EventDispatcher::getInstance().dispatchEvent(&e);

    delete *it;
    *it = NULL;
  }
  children.clear();
}

int DisplayObjectContainer::numChildren() {
  return children.size();
}

DisplayObject* DisplayObjectContainer::getChild(int index) {
  if (index > children.size())
    return nullptr;

  return children[index];
}

DisplayObject* DisplayObjectContainer::getChild(std::string id) {
  for (std::vector<DisplayObject*>::iterator it = children.begin();
       it != children.end(); it++) {
    if (id == (*it)->getID()) {
      return *it;
    }
  }
  return nullptr;
}

void DisplayObjectContainer::update() {
  DisplayObject::update();
}

void DisplayObjectContainer::draw(const glm::mat4& view_mat, const glm::mat4& proj_mat, 
                                  Shader& shader){
  for (std::vector<DisplayObject*>::iterator it = children.begin();
       it != children.end();) {
    if (*it == nullptr)
      it = children.erase(it);
    else
      ++it;
  }

  DisplayObject::draw(view_mat, proj_mat, shader);

  for (DisplayObject* child : children) {
    // shouldn't be possilbe but just in case
    if (child == nullptr)
      continue;

    child->draw(view_mat, proj_mat, shader);
  }

}

void DisplayObjectContainer::setRenderable(ShapeType type,
                                           const ShapeMetadata& meta) {
  DisplayObject::setRenderable(type, meta);
}

void DisplayObjectContainer::cleanUpDisplayTree() {
  for (DisplayObject* child : children) {
    if (child) {
      child->cleanUpDisplayTree();
      child->parent_ = NULL;
      delete child;
      child = nullptr;
    }
  }
  children.clear();
}
