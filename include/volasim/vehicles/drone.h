#ifndef DRONE_H
#define DRONE_H

#include <volasim/simulation/display_object_container.h>

class Drone : DisplayObjectContainer {
 public:
  Drone();
  Drone(std::string id);

  virtual ~Drone();

  virtual void update() override;
  virtual void draw() override;

 private:
};

#endif
