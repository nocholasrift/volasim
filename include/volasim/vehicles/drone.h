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

  Drone(const pugi::xml_node& root, double dt);
  Drone(const Eigen::Matrix3d& J_mat, double torque_const, double boom_length,
        double mass, double dt);

  virtual ~Drone() override;

  virtual void update(double dt) override;

  virtual void setTranslation(const glm::vec3& tran) override;
  virtual void setRotation(const glm::quat& rot) override;

  virtual void setVelocity(const glm::vec3& vel) override;
  virtual void setAngularVelocity(const glm::vec3& rpy) override;

  virtual void setInput(const Eigen::VectorXd& u) override;

  virtual void buildFromXML(const pugi::xml_node& root) override;

  virtual glm::vec3 getTranslation() override;
  virtual glm::quat getRotation() override;
  virtual glm::vec3 getVelocity() override;
  virtual glm::vec3 getBodyRates() override;

 private:
  std::unique_ptr<amrl::RungeKutta<N, M>> solver_;

  virtual Eigen::VectorXd dynamics(const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& u) override;

  double mass_;
  double boom_length_;
  double torque_const_;

  Eigen::Matrix3d J_mat_;
  Eigen::Matrix3d J_mat_inv_;

  X_t x_;
  U_t u_;
};

#endif
