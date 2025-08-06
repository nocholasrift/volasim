// from https://learnopengl.com/Getting-started/Camera
#ifndef CAMERA_H
#define CAMERA_H

#include <volasim/simulation/dynamic_object.h>

#include <glm/glm.hpp>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT };

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
 public:
  // camera Attributes

  Camera() = default;

  Camera(const glm::ivec2& window_sz, double yaw, double pitch, double radius,
         const glm::vec3& world_up, DynamicObject* target_obj = nullptr);

  // returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 getViewMatrix();

  void enableOrbitAndPan() { enable_orbit_and_pan_ = true; }

  void disableOrbitAndPan() { enable_orbit_and_pan_ = false; }

  // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
  void processKeyboard(Camera_Movement direction, float deltaTime);

  // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
  void processMouseMovement(float xoffset, float yoffset);

  // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
  void processMouseScroll(float yoffset);

  float getFov() { return fov_; }

 private:
  // calculates the front vector from the Camera's (updated) Euler Angles
  void updateCameraVectors();

 private:
  static constexpr float kPitchClipMargin = 0.01f;

  static constexpr float kMinRadius = 0.1f;
  static constexpr float kMaxRadius = 10.f;

  static constexpr float kMouseSense = .005f;
  static constexpr float kMouseWheelSense = .1f;

  glm::vec3 position_;
  glm::vec3 direction_;
  glm::vec3 up_;
  glm::vec3 right_;
  glm::vec3 world_up_;

  // camera options
  float movement_speed_;

  float fov_ = 90.f;
  float radius_ = 1.f;
  float yaw_ = 0.f;
  float pitch_ = 0.f;

  float lastx_;
  float lasty_;

  bool enable_orbit_and_pan_ = false;

  DynamicObject* target_obj_ = nullptr;
};
#endif
