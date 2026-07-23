#ifndef DISPLAYOBJECTCONTAINER_H
#define DISPLAYOBJECTCONTAINER_H

#include <volasim/simulation/display_object.h>
#include <fstream>
#include <string>
#include <vector>

class DisplayObjectContainer : public DisplayObject {

 public:
  DisplayObjectContainer();
  DisplayObjectContainer(std::string id);
  virtual ~DisplayObjectContainer();

  void addChild(DisplayObject* child);
  void removeChild(DisplayObject* child, bool should_delete = true);
  void removeChild(std::string id, bool should_delete = true);
  void removeChild(int index, bool should_delete = true);
  void removeAllChildren();

  int            numChildren();
  DisplayObject* getChild(int index);
  DisplayObject* getChild(std::string id);

  void update() override;
  void draw(const glm::mat4& view_mat, const glm::mat4& proj_mat,
            Shader& shader) override;

  void cleanUpDisplayTree() override;

  std::vector<DisplayObject*> children;

 private:
};

#endif
