#include <volasim/comms/msgs/Thrust.pb.h>
#include <volasim/vehicles/drone.h>

#include <glm/ext/quaternion_common.hpp>
#include <glm/gtc/quaternion.hpp>

#include <sstream>
#include <stdexcept>

Drone::Drone(double dt) : DynamicObject(dt) {

  x_ = Eigen::Matrix<double, Drone::N, 1>::Zero();
  u_ = Eigen::Matrix<double, Drone::M, 1>::Zero();

  // unit quaternion
  x_(3) = 1.0;

  // sporty drone, thrust to weight ratio of 3:1
  max_thrust_ = 3 * mass_ * 9.81;
}

Drone::Drone(const Eigen::Matrix3d& J_mat, double torque_const,
             double boom_length, double mass, double dt)
    : DynamicObject(dt) {
  x_ = Eigen::Matrix<double, Drone::N, 1>::Zero();
  u_ = Eigen::Matrix<double, Drone::M, 1>::Zero();

  // unit quaternion
  x_(3) = 1.0;

  setMass(mass);
  setInertia(J_mat);
  setBoomLength(boom_length);
  setTorqueConstant(torque_const);

  // sporty drone, thrust to weight ratio of 3:1
  max_thrust_ = 3 * mass_ * 9.81;
}

// Drone::Drone(std::string id) {}

Drone::~Drone() {}

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

  u_[0] = std::min(max_thrust_, u_[0]);
  u_[0] = std::max(0., u_[0]);
}

void Drone::setInput(const std::string& buffer) {
  volasim_msgs::Thrust msg;

  msg.ParseFromArray(buffer.data(), buffer.size());
  u_[0] = msg.f1();
  u_[1] = msg.f2();
  u_[2] = msg.f3();
  u_[3] = msg.f4();

  u_[0] = std::min(max_thrust_, u_[0]);
  u_[0] = std::max(0., u_[0]);

  u_.tail(3) = u_.tail(3).cwiseMin(max_thrust_ / 4. * boom_length_);
  u_.tail(3) = u_.tail(3).cwiseMax(-max_thrust_ / 4. * boom_length_);
}

void Drone::getForceAndTorque(Eigen::Vector3d& force, Eigen::Vector3d& torque) {

  Eigen::Quaterniond quat(x_[3], x_[4], x_[5], x_[6]);

  force = quat * Eigen::Vector3d(0, 0, u_[0]);
  torque = quat * u_.tail(3);
}

Drone* Drone::fromXML(const pugi::xml_node& root) {

  std::string mat_str = root.child_value("inertia_matrix");
  std::stringstream ss(mat_str);
  std::string value;

  Eigen::Matrix3d J_mat;

  int i = 0;
  while (ss >> value) {
    if (i > 8)
      throw std::runtime_error("[Drone] Intertia matrix has too many entries!");
    J_mat(i / 3, i % 3) = std::stof(value);
    i++;
  }

  if (i != 9)
    throw std::runtime_error(
        "[Drone] Inertia matrix must have exactly 9 entries, found " +
        std::to_string(i));

  return new Drone(J_mat, std::stof(root.child_value("c_torque")),
                   std::stof(root.child_value("length")),
                   std::stof(root.child_value("mass")), kSimulDT);
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

volasim_msgs::DroneState Drone::getSimState() {
  static int count = 0;
  volasim_msgs::DroneState state;

  state.mutable_odom()->mutable_position()->set_x(x_(0));
  state.mutable_odom()->mutable_position()->set_y(x_(1));
  state.mutable_odom()->mutable_position()->set_z(x_(2));

  state.mutable_odom()->mutable_orientation()->set_x(x_(4));
  state.mutable_odom()->mutable_orientation()->set_y(x_(5));
  state.mutable_odom()->mutable_orientation()->set_z(x_(6));
  state.mutable_odom()->mutable_orientation()->set_w(x_(3));

  state.mutable_odom()->mutable_linvel()->set_x(x_(7));
  state.mutable_odom()->mutable_linvel()->set_y(x_(8));
  state.mutable_odom()->mutable_linvel()->set_z(x_(9));

  state.mutable_odom()->mutable_angvel()->set_x(x_(10));
  state.mutable_odom()->mutable_angvel()->set_y(x_(11));
  state.mutable_odom()->mutable_angvel()->set_z(x_(12));

  state.mutable_imu()->mutable_orientation()->set_x(x_(4));
  state.mutable_imu()->mutable_orientation()->set_y(x_(5));
  state.mutable_imu()->mutable_orientation()->set_z(x_(6));
  state.mutable_imu()->mutable_orientation()->set_w(x_(3));

  state.mutable_imu()->mutable_angvel()->set_x(x_(10));
  state.mutable_imu()->mutable_angvel()->set_y(x_(11));
  state.mutable_imu()->mutable_angvel()->set_z(x_(12));

  Eigen::Vector3d force, torque;
  getForceAndTorque(force, torque);

  Eigen::Vector3d acceleration = force / mass_;
  state.mutable_imu()->mutable_linacc()->set_x(acceleration(0));
  state.mutable_imu()->mutable_linacc()->set_y(acceleration(1));
  state.mutable_imu()->mutable_linacc()->set_z(acceleration(2));

  /*(*state.mutable_odom()) = odom_msg;*/
  /*(*state.mutable_imu()) = imu_msg;*/

  return state;
}

// void Drone::draw() {}
