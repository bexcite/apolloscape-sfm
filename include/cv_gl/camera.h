// Copyright Pavlo 2018
#ifndef CV_GL_CAMERA_H_
#define CV_GL_CAMERA_H_

#include <glm/glm.hpp>

enum CameraMovement {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,

  MOVE_ORIGIN,
  MOVE_TOP,
  MOVE_SIDEWAYS
};

// camera constant
const float kYaw = 0.0f;
const float kPitch = 5.0f;  // -5.0f
const float kSpeed = 2.5f;
const float kSensitivity = 0.1f;
const float kZoom = 45.0f;

class Camera {
 public:
  Camera(const glm::vec3& position = glm::vec3(0.0f, 0.0f, 0.0f),
      const glm::vec3& world_up = glm::vec3(0.0f, 0.0f, 1.0f),
      float yaw = kYaw, float pitch = kPitch);

  void ProcessKeyboard(CameraMovement camera_movement, float delta_time);
  void ProcessMouseInput(float xoffset, float yoffset, bool constrain_pitch = true);
  void ProcessMouseScroll(float yoffset);

  glm::mat4 GetViewMatrix() const;
  glm::mat4 GetProjMatrix() const;
  float GetZoom() const;

  void SetDirection(const glm::vec3& direction_to);
  void SetPosition(const glm::vec3& position);

  void SetIntrinsics(const float fx, const float fy, const float cx,
      const float cy);


 private:

   // Intrinsics
   float fx_ = 1450.317230113;
   float fy_ =1451.184836113;
   float cx_ = 1244.386581025;
   float cy_ = 1013.145997723;

   float image_width_ = 2452;
   float image_height_ = 2056;

   // float width = 1.0f;
   // float height = 1.0f;

   float near_ = 0.1;
   float far_ = 100;

   // Extrinsics
   glm::vec3 position_;
   glm::vec3 front_;
   glm::vec3 up_;
   glm::vec3 right_;
   glm::vec3 world_up_;

   float yaw_;
   float pitch_;

   float movement_speed_;
   float mouse_sensitivity_;

   // TODO (Pavlo): Should be removed probably
   float zoom_;

   void UpdateCameraVectors(bool constrain_pitch = true);

};

#endif  // CV_GL_CAMERA_H_
