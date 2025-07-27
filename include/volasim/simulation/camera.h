// from https://learnopengl.com/Getting-started/Camera
#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT };

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
 public:
  // camera Attributes
  glm::vec3 Position;
  glm::vec3 Front;
  glm::vec3 Up;
  glm::vec3 Right;
  glm::vec3 WorldUp;
  // euler Angles
  float yaw_;
  float pitch_;
  // camera options
  float movement_speed_;
  float mouse_sensitivity_;
  float zoom_;
  float lastx_;
  float lasty_;

  Camera() = default;

  // constructor with vectors
  Camera(glm::ivec2 window_sz, glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
         glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = kYaw,
         float pitch = kPitch)
      : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
        movement_speed_(kSpeed),
        mouse_sensitivity_(kSensitivity),
        zoom_(kZoom) {
    Position = position;
    WorldUp = up;
    yaw_ = yaw;
    pitch_ = pitch;
    lastx_ = static_cast<float>(window_sz[0]) / 2.;
    lasty_ = static_cast<float>(window_sz[1]) / 2.;
    updateCameraVectors();
  }

  // constructor with scalar values
  Camera(int width, int height, float posX, float posY, float posZ, float upX,
         float upY, float upZ, float yaw, float pitch)
      : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
        movement_speed_(kSpeed),
        mouse_sensitivity_(kSensitivity),
        zoom_(kZoom) {
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    yaw_ = yaw;
    pitch_ = pitch;
    lastx_ = static_cast<float>(width) / 2.;
    lasty_ = static_cast<float>(height) / 2.;
    updateCameraVectors();
  }

  // returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 getViewMatrix() {
    // std::cout << Position[0] << " " << Position[1] << " " << Position[2] << "\n"
    //           << WorldUp[0] << " " << WorldUp[1] << " " << WorldUp[2] << " "
    //           << yaw_ << " " << pitch_ << std::endl;
    return glm::lookAt(Position, Position + Front, Up);
  }

  // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
  void processKeyboard(Camera_Movement direction, float deltaTime) {}

  // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
  void processMouseMovement(float xoffset, float yoffset,
                            bool constrainPitch = true) {
    // float xoffset = x - lastx_;
    // float yoffset =
    //     lasty_ - y;  // reversed since y-coordinates range from bottom to top

    xoffset *= mouse_sensitivity_;
    yoffset *= mouse_sensitivity_;

    yaw_ += xoffset;
    pitch_ += -yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainPitch) {
      if (pitch_ > 89.0f)
        pitch_ = 89.0f;
      if (pitch_ < -89.0f)
        pitch_ = -89.0f;
    }

    // lastx_ = x;
    // lasty_ = y;

    // update Front, Right and Up Vectors using the updated Euler angles
    updateCameraVectors();
  }

  // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
  void processMouseScroll(float yoffset) {
    zoom_ -= (float)yoffset;
    if (zoom_ < 1.0f)
      zoom_ = 1.0f;
    if (zoom_ > 45.0f)
      zoom_ = 45.0f;
  }

 private:
  static constexpr float kYaw = -90.0f;
  static constexpr float kPitch = 0.0f;
  static constexpr float kSpeed = 2.5f;
  static constexpr float kSensitivity = 0.1f;
  static constexpr float kZoom = 45.0f;

  // calculates the front vector from the Camera's (updated) Euler Angles
  void updateCameraVectors() {
    // calculate the new Front vector
    glm::vec3 front;
    front.x = cos(glm::radians(yaw_)) * cos(glm::radians(pitch_));
    front.y = sin(glm::radians(pitch_));
    front.z = sin(glm::radians(yaw_)) * cos(glm::radians(pitch_));
    Front = glm::normalize(front);
    // also re-calculate the Right and Up vector
    Right = glm::normalize(glm::cross(
        Front,
        WorldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    Up = glm::normalize(glm::cross(Right, Front));
  }
};
#endif
