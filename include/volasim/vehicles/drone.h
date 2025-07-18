#ifndef DRONE_H
#define DRONE_H

#include <string>
#include <volasim/simulation/dynamic_object.h>

#define N_state 13
#define M_input 4

class Drone : public DynamicObject<N_state, M_input>{
 public:

  using X_t = Eigen::Vector<double, N_state>;
  using U_t = Eigen::Vector<double, M_input>;

  Drone(const Eigen::Matrix3d& J_mat, double torque_const, double boom_length, double mass, double dt);

  virtual ~Drone();

  virtual void update(const U_t& u, double dt) override;

  virtual void setTranslation(const glm::vec3& tran) override;
  virtual void setRotation(const glm::quat& rot) override;

  virtual glm::vec3 getTranslation() override;
  virtual glm::quat getRotation() override;
  virtual glm::vec3 getVelocity() override;
  virtual glm::vec3 getBodyRates() override;

 private:

  Eigen::Vector<double, N_state> dynamics(const X_t& x, const U_t& u) override;

  double mass_;
  double boom_length_;
  double torque_const_;

  Eigen::Matrix3d J_mat_;
  Eigen::Matrix3d J_mat_inv_;

  X_t x_;
  U_t u_;
};

#endif
