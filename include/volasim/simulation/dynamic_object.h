#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <memory>
#include <regex>

#include <volasim/simulation/display_object.h>
#include <volasim/simulation/simulation.h>
#include <volasim/solvers/runge_kutta.h>

class DynamicObject {
 public:
  DynamicObject(double dt) : dt_(dt) {}
  virtual ~DynamicObject() {}

  // dynamics function
  virtual void update(double dt) = 0;

  virtual Eigen::VectorXd dynamics(
      const Eigen::VectorXd& x, const Eigen::VectorXd& u) = 0;

  virtual void setTranslation(const glm::vec3& tran) = 0;
  virtual void setRotation(const glm::quat& rot) = 0;

  virtual void setInput(const Eigen::VectorXd& u) = 0;

  virtual glm::vec3 getTranslation() = 0;
  virtual glm::quat getRotation() = 0;

  virtual glm::vec3 getVelocity() = 0;
  virtual glm::vec3 getBodyRates() = 0;

 protected:
  double dt_;
};

// class DynamicDisplayWrapper {
//  public:
//   DynamicDisplayWrapper(DisplayObject* display_obj,
//                         std::unique_ptr<DynamicObject>& obj)
//       : display_obj_(display_obj), obj_(std::move(obj_)) {
//
//   }
//
//   ~DynamicDisplayWrapper() {}
//
//   void update(const Eigen::VectorXd& u, double dt) {
//     obj_->update(u, dt);
//
//     display_obj_->setTranslation(obj_->getTranslation());
//     display_obj_->setRotation(obj_->getRotation());
//     display_obj_->update();
//   }
//
//   DynamicObject& getDynaObj() { return *obj_; }
//
//  protected:
//   std::unique_ptr<DynamicObject> obj_;
//   DisplayObject* display_obj_;
// };

#endif
