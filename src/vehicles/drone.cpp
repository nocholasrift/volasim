#include <volasim/comms/msgs/Thrust.pb.h>
#include <volasim/vehicles/drone.h>

#include <glm/ext/quaternion_common.hpp>
#include <glm/gtc/quaternion.hpp>

#include <sstream>
#include <stdexcept>

Drone::Drone(const pugi::xml_node& root, double dt) : DynamicObject(dt) {
  buildFromXML(root);

  x_ = Eigen::Matrix<double, Drone::N, 1>::Zero();
  u_ = Eigen::Matrix<double, Drone::M, 1>::Zero();

  // unit quaternion
  x_(3) = 1.0;

  solver_ = std::make_unique<amrl::RungeKutta<Drone::N, Drone::M>>(
      [this](const X_t& x, const U_t& u) { return this->dynamics(x, u); });
}

Drone::Drone(const Eigen::Matrix3d& J_mat, double torque_const,
             double boom_length, double mass, double dt)
    : DynamicObject(dt) {
  x_ = Eigen::Matrix<double, Drone::N, 1>::Zero();
  u_ = Eigen::Matrix<double, Drone::M, 1>::Zero();

  // unit quaternion
  x_(3) = 1.0;

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
  x_ = solver_->step(x_, u_, dt);

  // Re-normalize quaternion (w, x, y, z) to avoid drift
  glm::quat q(x_[3], x_[4], x_[5], x_[6]);
  q = glm::normalize(q);

  // just in case any nans pop up
  if (glm::any(glm::isnan(glm::vec4(q.x, q.y, q.z, q.w)))) {
    q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  }

  x_(3) = q.w;
  x_(4) = q.x;
  x_(5) = q.y;
  x_(6) = q.z;
}

void Drone::setTranslation(const glm::vec3& tran) {
  x_(0) = tran[0];
  x_(1) = tran[1];
  x_(2) = tran[2];
}

void Drone::setRotation(const glm::quat& rot) {
  x_(3) = rot[3];
  x_(4) = rot[0];
  x_(5) = rot[1];
  x_(6) = rot[2];
}

void Drone::setVelocity(const glm::vec3& vel) {
  x_(7) = vel[0];
  x_(8) = vel[1];
  x_(9) = vel[2];
}

void Drone::setAngularVelocity(const glm::vec3& rpy) {
  x_(10) = rpy[0];
  x_(11) = rpy[1];
  x_(12) = rpy[2];
}

void Drone::setInput(const Eigen::VectorXd& u) {
  u_ = u;
}

void Drone::setInput(const std::string& buffer) {
  volasim_msgs::Thrust msg;

  msg.ParseFromArray(buffer.data(), buffer.size());
  u_[0] = msg.f1();
  u_[1] = msg.f2();
  u_[2] = msg.f3();
  u_[3] = msg.f4();
}

void Drone::buildFromXML(const pugi::xml_node& root) {
  mass_ = std::stof(root.child_value("mass"));
  boom_length_ = std::stof(root.child_value("length"));
  torque_const_ = std::stof(root.child_value("c_torque"));

  std::string mat_str = root.child_value("inertia_matrix");
  std::stringstream ss(mat_str);
  std::string value;

  int i = 0;
  while (ss >> value) {
    if (i > 8)
      throw std::runtime_error("[Drone] Intertia matrix has too many entries!");
    J_mat_(i / 3, i % 3) = std::stof(value);
    i++;
  }

  if (i != 9)
    throw std::runtime_error(
        "[Drone] Inertia matrix must have exactly 9 entries, found " +
        std::to_string(i));

  J_mat_inv_ = J_mat_.inverse();
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
  return glm::quat(x_[3], x_[4], x_[5], x_[6]);
}

volasim_msgs::Odometry Drone::getSimState() {
  static int count = 0;
  volasim_msgs::Odometry odom_msg;

  odom_msg.mutable_position()->set_x(x_(0));
  odom_msg.mutable_position()->set_y(x_(1));
  odom_msg.mutable_position()->set_z(x_(2));

  odom_msg.mutable_orientation()->set_x(x_(4));
  odom_msg.mutable_orientation()->set_y(x_(5));
  odom_msg.mutable_orientation()->set_z(x_(6));
  odom_msg.mutable_orientation()->set_w(x_(3));

  odom_msg.mutable_linvel()->set_x(x_(7));
  odom_msg.mutable_linvel()->set_y(x_(8));
  odom_msg.mutable_linvel()->set_z(x_(9));

  odom_msg.mutable_angvel()->set_x(x_(10));
  odom_msg.mutable_angvel()->set_y(x_(11));
  odom_msg.mutable_angvel()->set_z(x_(12));

  return odom_msg;
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

  glm::vec4 f(u(0), u(1), u(2), u(3));
  glm::vec3 thrust_vec(0, 0, (u(0) + u(1) + u(2) + u(3)) / mass_);

  glm::vec3 v_dot = glm::vec3(0, 0, -9.81) + R * thrust_vec;

  glm::vec3 R_thrust = R * thrust_vec;

  // rigid body dynamics for w_dot
  Eigen::Vector3d w_eig(x(10), x(11), x(12));
  Eigen::Vector3d torque(boom_length_ * (u[0] + u[1] - u[2] - u[3]) / sqrt(2),
                         boom_length_ * (-u[0] + u[1] + u[2] - u[3]) / sqrt(2),
                         torque_const_ * (u[0] - u[1] + u[2] - u[3]));
  Eigen::Vector3d w_dot = J_mat_inv_ * (torque - w_eig.cross(J_mat_ * w_eig));

  X_t x_dot;
  x_dot(0) = v[0],      // pos X
      x_dot(1) = v[1],  // pos Y
      x_dot(2) = v[2],  // pos Z
      x_dot(3) = q_dot[3], x_dot(4) = q_dot[0], x_dot(5) = q_dot[1],
  x_dot(6) = q_dot[2], x_dot(7) = v_dot[0], x_dot(8) = v_dot[1],
  x_dot(9) = v_dot[2], x_dot(10) = w_dot[0], x_dot(11) = w_dot[1],
  x_dot(12) = w_dot[2];

  return x_dot;
}

// void Drone::draw() {}
