// Copyright Pavlo 2018
#ifndef CV_GL_CAMERA_H_
#define CV_GL_CAMERA_H_

#include <glm/glm.hpp>

enum CameraMovement {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

// camera constant
const float kYaw = 90.0f;
const float kPitch = -5.0f;
const float kSpeed = 2.5f;
const float kSensitivity = 0.1f;
const float kZoom = 45.0f;

class Camera {
 public:
  Camera(const glm::vec3& position = glm::vec3(0.0f, 0.0f, 0.0f),
      const glm::vec3& world_up = glm::vec3(0.0f, 1.0f, 0.0f),
      float yaw = kYaw, float pitch = kPitch);

  void ProcessKeyboard(CameraMovement camera_movement, float delta_time);
  void ProcessMouseInput(float xoffset, float yoffset, bool constrain_pitch = true);
  void ProcessMouseScroll(float yoffset);

  glm::mat4 GetViewMatrix() const;
  float GetZoom() const;

  void SetDirection(const glm::vec3& direction_to);
  void SetPosition(const glm::vec3& position);


 private:
   glm::vec3 position_;
   glm::vec3 front_;
   glm::vec3 up_;
   glm::vec3 right_;
   glm::vec3 world_up_;

   float yaw_;
   float pitch_;

   float movement_speed_;
   float mouse_sensitivity_;
   float zoom_;

   void UpdateCameraVectors(bool constrain_pitch = true);

};

#endif  // CV_GL_CAMERA_H_
