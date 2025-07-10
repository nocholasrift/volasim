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

  int numChildren();
  DisplayObject* getChild(int index);
  DisplayObject* getChild(std::string id);

  virtual void update() override;
  virtual void draw() override;
  virtual void cleanUpDisplayTree() override;

  std::vector<DisplayObject*> children;

 private:
};

#endif
