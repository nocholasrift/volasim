#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <pugixml.hpp>

#include <memory>
#include <regex>

#include <volasim/comms/msgs/Odometry.pb.h>
#include <volasim/simulation/display_object.h>
#include <volasim/solvers/runge_kutta.h>

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

  virtual void setMass(double mass) {
    if (mass < 1e-6) {
      throw std::invalid_argument("[DynamicObject] mass must be > 0");
    }
    mass_ = mass;
  }

  virtual void setInertia(const Eigen::Matrix3d& J) {
    const Eigen::Matrix3d sym = (J + J.transpose()) / 2.;
    Eigen::LLT<Eigen::Matrix3d> llt(sym);

    if (llt.info() != Eigen::Success) {
      throw std::invalid_argument(
          "DynamicObject::setInertia: inertia must be symmetric "
          "positive-definite");
    }

    J_mat_ = sym;
    J_mat_inv_ = llt.solve(Eigen::Matrix3d::Identity());

    std::cout << J_mat_;
  }

  virtual void buildFromXML(const pugi::xml_node& root) = 0;

  virtual volasim_msgs::Odometry getSimState() = 0;

  virtual glm::vec3 getTranslation() = 0;
  virtual glm::quat getRotation() = 0;

  virtual glm::vec3 getVelocity() = 0;
  virtual glm::vec3 getBodyRates() = 0;

  virtual double getMass() { return mass_; }
  virtual Eigen::Matrix3d getInertia() { return J_mat_; }
  virtual Eigen::Matrix3d getInertiaInv() { return J_mat_inv_; }

  virtual void getForceAndTorque(Eigen::Vector3d& force,
                                 Eigen::Vector3d& torque) = 0;

 protected:
  double dt_;

  double mass_;
  Eigen::Matrix3d J_mat_;
  Eigen::Matrix3d J_mat_inv_;
};

#endif
