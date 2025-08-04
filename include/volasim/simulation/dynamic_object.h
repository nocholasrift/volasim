#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <pugixml.hpp>

#include <memory>
#include <regex>

#include <volasim/simulation/display_object.h>
#include <volasim/solvers/runge_kutta.h>
#include <volasim/comms/msgs/Odometry.pb.h>

class DynamicObject {
 public:
  DynamicObject(double dt) : dt_(dt) {}

  virtual ~DynamicObject() {}

  // dynamics function
  virtual void update(double dt) = 0;

  virtual Eigen::VectorXd dynamics(const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& u) = 0;

  virtual void setTranslation(const glm::vec3& tran) = 0;
  virtual void setRotation(const glm::quat& rot) = 0;
  virtual void setVelocity(const glm::vec3& vel) = 0;
  virtual void setAngularVelocity(const glm::vec3& rpy) = 0;
  virtual void setInput(const Eigen::VectorXd& u) = 0;
  virtual void setInput(const std::string& buffer) = 0;

  virtual void buildFromXML(const pugi::xml_node& root) = 0;

  virtual volasim_msgs::Odometry getSimState() = 0;

  virtual glm::vec3 getTranslation() = 0;
  virtual glm::quat getRotation() = 0;

  virtual glm::vec3 getVelocity() = 0;
  virtual glm::vec3 getBodyRates() = 0;

 protected:
  double dt_;
};


#endif
