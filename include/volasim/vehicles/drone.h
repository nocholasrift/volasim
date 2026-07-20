#ifndef DRONE_H
#define DRONE_H

#include <volasim/simulation/dynamic_object.h>
#include <string>

class Drone : public DynamicObject {
 public:
  static constexpr unsigned int N = 13;
  static constexpr unsigned int M = 4;

  using X_t = Eigen::Matrix<double, N, 1>;
  using U_t = Eigen::Matrix<double, M, 1>;

  Drone(double dt);
  Drone(const Eigen::Matrix3d& J_mat, double torque_const, double boom_length,
        double mass, double dt);

  static Drone* fromXML(const pugi::xml_node& node);

  virtual ~Drone() override;

  virtual void setTranslation(const glm::vec3& tran) override;
  virtual void setRotation(const glm::quat& rot) override;

  virtual void setVelocity(const glm::vec3& vel) override;
  virtual void setAngularVelocity(const glm::vec3& rpy) override;

  virtual void setInput(const Eigen::VectorXd& u) override;
  virtual void setInput(const std::string& buffer) override;

  void setBoomLength(double length) { boom_length_ = length; }
  void setTorqueConstant(double torque) { torque_const_ = torque; }

  virtual volasim_msgs::DroneState getSimState() override;
  virtual glm::vec3 getTranslation() override;
  virtual glm::quat getRotation() override;
  virtual glm::vec3 getVelocity() override;
  virtual glm::vec3 getBodyRates() override;

  virtual void getForceAndTorque(Eigen::Vector3d& force,
                                 Eigen::Vector3d& torque) override;

 private:
  double boom_length_{0.75};
  double torque_const_{0.1};

  double max_thrust_{1.0};

  static constexpr float kSimulDT = 1. / 60.;

  X_t x_;
  U_t u_;
};

#endif
