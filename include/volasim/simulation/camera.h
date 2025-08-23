// from https://learnopengl.com/Getting-started/Camera
#ifndef CAMERA_H
#define CAMERA_H

#include <volasim/simulation/dynamic_object.h>

#include <glm/glm.hpp>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT };

struct CameraSettings {
  float yaw = 0.f;
  float pitch = 0.f;
  float radius = 1.f;
  float fov = 60.f;

  float fps = 60.f;

  glm::ivec2 window_sz = glm::ivec2(640, 480);

  DynamicObject* target = nullptr;
};

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
 public:
  // camera Attributes

  Camera() = default;

  Camera(const CameraSettings& settings);

  Camera(const glm::ivec2& window_sz, double yaw, double pitch, double radius,
         double fov = 60.f,
         const glm::vec3& world_up = glm::vec3(0.f, 0.f, 1.f),
         DynamicObject* target_obj = nullptr);

  // returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 getViewMatrix();

  float getFov() { return fov_; }

  void setTarget(DynamicObject* target_obj) { target_obj_ = target_obj; }

  void enableOrbitAndPan() { enable_orbit_and_pan_ = true; }

  void disableOrbitAndPan() { enable_orbit_and_pan_ = false; }

  // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
  void processKeyboard(Camera_Movement direction, float deltaTime);

  // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
  void processMouseMovement(float xoffset, float yoffset);

  // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
  void processMouseScroll(float yoffset);

 private:
  // calculates the front vector from the Camera's (updated) Euler Angles
  void updateCameraVectors();

 private:
  static constexpr float kPitchClipMargin = 0.01f;

  static constexpr float kMinRadius = 0.1f;
  static constexpr float kMaxRadius = 100.f;

  static constexpr float kMouseSense = .005f;
  static constexpr float kMouseWheelSense = .5f;

  glm::vec3 position_;
  glm::vec3 direction_;
  glm::vec3 up_;
  glm::vec3 right_;
  glm::vec3 world_up_ = glm::vec3(0.f, 0.f, 1.f);

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
