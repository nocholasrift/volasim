// from https://learnopengl.com/Getting-started/Camera
#ifndef CAMERA_H
#define CAMERA_H

#include <volasim/simulation/dynamic_object.h>

#include <glm/glm.hpp>
#include <pugixml.hpp>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT };

struct CameraSettings {
  float yaw    = 0.F;
  float pitch  = 0.F;
  float radius = 1.F;
  float fov    = 60.F;

  float fps = 60.F;

  glm::ivec2 window_sz = glm::ivec2(640, 480);

  DynamicObject* target = nullptr;
};

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
 public:
  // camera Attributes
  struct Dimensions {
    int width{0};
    int height{0};
  };

  Camera() = default;

  Camera(const CameraSettings& settings);

  Camera(const glm::ivec2& window_sz, double yaw, double pitch, double radius,
         double fov = 60.F, double fps = 60.F,
         const glm::vec3& world_up   = glm::vec3(0.F, 0.F, 1.F),
         DynamicObject*   target_obj = nullptr);

  static Camera fromXML(const pugi::xml_node& camera_xml);

  // returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 getViewMatrix();

  [[nodiscard]] float getFov() const { return fov_; }

  [[nodiscard]] unsigned int getFPS() const { return fps_; }

  Dimensions getDimensions() { return dimensions_; }

  void setTarget(DynamicObject* target_obj) { target_obj_ = target_obj; }

  void                  setID(uint8_t id) { id_ = id; }
  [[nodiscard]] uint8_t getID() const { return id_; }

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
  static constexpr float kPitchClipMargin = 0.01F;

  static constexpr float kMinRadius = 0.1F;
  static constexpr float kMaxRadius = 100.F;

  static constexpr float kMouseSense      = .005F;
  static constexpr float kMouseWheelSense = .5F;

  glm::vec3 position_;
  glm::vec3 direction_;
  glm::vec3 up_;
  glm::vec3 right_;
  glm::vec3 world_up_ = glm::vec3(0.F, 0.F, 1.F);

  uint8_t id_{0};

  // camera options
  float movement_speed_;

  float fov_    = 90.F;
  float radius_ = 1.F;
  float yaw_    = 0.F;
  float pitch_  = 0.F;

  float lastx_{0.};
  float lasty_{0.};

  unsigned int fps_ = 60;

  Dimensions dimensions_;

  bool enable_orbit_and_pan_ = false;

  DynamicObject* target_obj_ = nullptr;
};
#endif
