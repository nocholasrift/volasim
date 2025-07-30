#include <volasim/vehicles/drone.h>
#include <glm/ext/quaternion_common.hpp>
#include <glm/gtc/quaternion.hpp>

Drone::Drone(const Eigen::Matrix3d& J_mat, double torque_const,
             double boom_length, double mass, double dt)
    : DynamicObject(dt) {
  x_ = Eigen::Matrix<double, Drone::N, 1>::Zero();
  u_ = Eigen::Matrix<double, Drone::M, 1>::Zero();

  // unit quaternion
  x_(6) = 1.0;

  mass_ = mass;
  torque_const_ = torque_const;

  J_mat_ = J_mat;
  J_mat_inv_ = J_mat.inverse();
  boom_length_ = boom_length;

  solver_ = std::make_unique<amrl::RungeKutta<Drone::N, Drone::M>>(
      [this](const X_t& x, const U_t& u) { return this->dynamics(x, u); });
}

// Drone::Drone(std::string id) {}

Drone::~Drone() {}

void Drone::update(double dt) {
  // std::cout << "input is: " << u_.transpose() << std::endl;
  // std::cout << "x_old: " << x_.transpose() << std::endl;
  x_ = solver_->step(x_, u_, dt);
  // std::cout << "x_new: " << x_.transpose() << std::endl;
  // std::cout << "done with step" << std::endl;
}

void Drone::setTranslation(const glm::vec3& tran) {
  x_(0) = tran[0];
  x_(1) = tran[1];
  x_(2) = tran[2];
}

void Drone::setRotation(const glm::quat& rot) {
  x_(6) = rot[0];
  x_(3) = rot[1];
  x_(4) = rot[2];
  x_(5) = rot[3];
}

void Drone::setInput(const Eigen::VectorXd& u) {
  u_ = u;
}

glm::vec3 Drone::getVelocity() {
  return glm::vec3(x_(7), x_(8), x_(9));
}

glm::vec3 Drone::getBodyRates() {
  return glm::vec3(x_(10), x_(11), x_(12));
}

glm::vec3 Drone::getTranslation() {
  return glm::vec3(x_[0], x_[1], x_[2]);
}

glm::quat Drone::getRotation() {
  return glm::quat(x_[6], x_[3], x_[4], x_[5]);
}

Eigen::VectorXd Drone::dynamics(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& u) {

  glm::vec3 p(x(0), x(1), x(2));
  glm::vec3 v(x(7), x(8), x(9));
  glm::vec3 w(x(10), x(11), x(12));

  glm::quat q(x(3), x(4), x(5), x(6));
  glm::mat3 R = glm::mat3_cast(q);

  glm::quat omega_q(0, w[0], w[1], w[2]);
  glm::quat q_dot = 0.5f * q * omega_q;
  // glm::vec4 q_dot(-1*(w[0]*q[1]+w[1]*q[2]+w[2]*q[3])/2.,
  //                 (w[0]*q[0]+w[1]*q[3]-w[2]*q[2])/2.,
  //                 (w[1]*q[0]+w[2]*q[1]-w[0]*q[3])/2.,
  //                 (w[2]*q[0]+w[0]*q[2]-w[1]*q[1])/2.);

  glm::vec4 f(u(0), u(1), u(2), u(3));
  glm::vec3 thrust_vec(0, 0, (u(0) + u(1) + u(2) + u(3)) / mass_);

  glm::vec3 v_dot = glm::vec3(0, 0, -9.81) + R * thrust_vec;

  // std::cout << "quat: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
  //           << std::endl;
  // for (int row = 0; row < 3; ++row) {
  //   for (int col = 0; col < 3; ++col) {
  //     std::cout << R[col][row] << " ";
  //   }
  //   std::cout << std::endl;
  // }

  glm::vec3 R_thrust = R * thrust_vec;
  // std::cout << "v " << R_thrust[0] << " " << R_thrust[1] << " " << R_thrust[2]
  //           << std::endl;
  // std::cout << "calculating v_dot " << v_dot[0] << " " << v_dot[1] << " "
  //           << v_dot[2] << std::endl;

  // rigid body dynamics for w_dot
  Eigen::Vector3d w_eig(x(10), x(11), x(12));
  Eigen::Vector3d torque(boom_length_ * (u[0] + u[1] - u[2] - u[3]) / sqrt(2),
                         boom_length_ * (-u[0] + u[1] + u[2] - u[3]) / sqrt(2),
                         torque_const_ * (u[0] - u[1] + u[2] - u[3]));
  Eigen::Vector3d w_dot = J_mat_inv_ * (torque - w_eig.cross(J_mat_ * w_eig));

  // std::cout << torque.transpose() << std::endl;
  // std::cout << "w_dot: " << w_dot.transpose() << std::endl;

  X_t x_dot;
  x_dot(0) = v[0],      // pos X
      x_dot(1) = v[1],  // pos Y
      x_dot(2) = v[2],  // pos Z
      x_dot(3) = q_dot[0], x_dot(4) = q_dot[1], x_dot(5) = q_dot[2],
  x_dot(6) = q_dot[3], x_dot(7) = v_dot[0], x_dot(8) = v_dot[1],
  x_dot(9) = v_dot[2], x_dot(10) = w_dot[0], x_dot(11) = w_dot[1],
  x_dot(12) = w_dot[2];

  // std::cout << "x_dot: " << x_dot.transpose() << std::endl;

  return x_dot;
}

// void Drone::draw() {}
