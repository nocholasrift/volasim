#include <volasim/simulation/camera.h>

Camera::Camera(const glm::ivec2& window_sz, double yaw, double pitch,
               double radius, double fov, unsigned int fps,
               const glm::vec3& world_up, DynamicObject* target_obj)
    : world_up_(world_up),
      fov_(static_cast<float>(fov)),
      radius_(static_cast<float>(radius)),
      yaw_(static_cast<float>(yaw)),
      pitch_(static_cast<float>(pitch)),
      lastx_(static_cast<float>(window_sz[0]) / 2.F),
      lasty_(static_cast<float>(window_sz[1]) / 2.F),
      fps_(fps),
      dimensions_{window_sz[0], window_sz[1]},
      target_obj_(target_obj) {
  updateCameraVectors();
}

Camera::Camera(const CameraSettings& settings)
    : fov_(settings.fov),
      radius_(settings.radius),
      yaw_(settings.yaw),
      pitch_(settings.pitch),
      lastx_(static_cast<float>(settings.window_sz[0]) / 2.F),
      lasty_(static_cast<float>(settings.window_sz[1]) / 2.F),
      fps_(settings.fps),
      dimensions_{settings.window_sz[0], settings.window_sz[1]},
      target_obj_(settings.target) {
  updateCameraVectors();
}

Camera Camera::fromXML(const pugi::xml_node& camera_xml) {
  CameraSettings settings;

  try {
    float width  = std::stof(camera_xml.child_value("window_width"));
    float height = std::stof(camera_xml.child_value("window_height"));

    settings.window_sz = glm::ivec2(width, height);
    settings.yaw       = std::stof(camera_xml.child_value("cam_yaw"));
    settings.pitch     = std::stof(camera_xml.child_value("cam_pitch"));
    settings.radius    = std::stof(camera_xml.child_value("cam_distance"));
    settings.fov       = std::stof(camera_xml.child_value("fov_deg"));
    settings.fps =
        static_cast<unsigned int>(std::stoul(camera_xml.child_value("fps")));

    settings.target = nullptr;
  } catch (const std::exception& e) {
    throw std::runtime_error("[XMLParser] Invalid camera settings in XML: " +
                             std::string(e.what()));
  }

  return Camera(settings);
}

glm::mat4 Camera::getViewMatrix() {
  if (target_obj_ == nullptr) {
    return glm::lookAt(position_, glm::vec3(0.f, 0.f, 0.f), up_);
  }

  return glm::lookAt(position_ + target_obj_->getTranslation(),
                     target_obj_->getTranslation(), up_);
}

void Camera::processKeyboard(Camera_Movement direction, float deltaTime) {}

void Camera::processMouseMovement(float xoffset, float yoffset) {
  if (!enable_orbit_and_pan_) {
    return;
  }

  yaw_   -= (xoffset * kMouseSense);
  pitch_ += (yoffset * kMouseSense);

  if (pitch_ > M_PI / 2 - kPitchClipMargin)
    pitch_ = M_PI / 2 - kPitchClipMargin;
  if (pitch_ < -M_PI / 2 + kPitchClipMargin)
    pitch_ = -M_PI / 2 + kPitchClipMargin;

  updateCameraVectors();
}

void Camera::processMouseScroll(float yoffset) {
  radius_ -= static_cast<float>(yoffset * kMouseWheelSense);

  if (radius_ < kMinRadius)
    radius_ = kMinRadius;
  else if (radius_ > kMaxRadius)
    radius_ = kMaxRadius;

  updateCameraVectors();
}

void Camera::updateCameraVectors() {

  // spherical coordinates, found to be much more stable than
  // glm mat4 multiplication
  position_[0] = radius_ * cos(pitch_) * cos(yaw_);
  position_[1] = radius_ * cos(pitch_) * sin(yaw_);
  position_[2] = radius_ * sin(pitch_);

  direction_ = glm::normalize(position_);
  right_     = glm::normalize(glm::cross(world_up_, direction_));

  up_ = glm::cross(direction_, right_);
}
