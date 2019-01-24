// Copyright Pavlo 2018
#ifndef CV_GL_CAMERA_H_
#define CV_GL_CAMERA_H_

#include <iostream>

#include <glm/glm.hpp>

enum CameraMovement
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,

  MOVE_ORIGIN,
  MOVE_TOP,
  MOVE_SIDEWAYS_RIGHT,
  MOVE_SIDEWAYS_LEFT
};

struct CameraIntrinsics {
  float fx;
  float fy;
  float s;
  float cx;
  float cy;
  float wr;
  glm::mat3 GetCameraMatrix() const {
    glm::mat3 camera_matrix(1.0);
    camera_matrix[0] = {fx /* *wr */, 0.0, 0.0};
    camera_matrix[1] = {0.0, fy, 0.0};
    camera_matrix[2] = {cx /* *wr*/, cy, 1.0};
    return camera_matrix;
  }
  void print(std::ostream& os = std::cout) const {
    os << "(fx = " << fx << ", ";
    os << "fy = " << fy << ", ";
    os << "cx = " << cx << ", ";
    os << "cy = " << cy << ", ";
    os << "wr = " << wr << ")";
  }
};

std::ostream& operator<<(std::ostream& os, const CameraIntrinsics& intr);

// camera constant
const float kYaw = 0.0f;
const float kPitch = 0.0f;  // -5.0f    5.0f
const float kSpeed = 10.5f; // 2.5f
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
  void SetOrigin(const glm::vec3& origin);
  void SetRotation(const float x_angle, const float y_angle, const float z_angle);
  void SetScale(const float scale);

  void Print(std::ostream& os = std::cout) const;

  void SetIntrinsics(const float fx, const float fy, const float cx,
      const float cy, const float wr);

  void SetIntrinsics(const CameraIntrinsics& camera_intr);

  float GetImageWidth() { return image_width_; }
  float GetImageHeight() { return image_height_; }

  float GetCx() const { return cx_; }
  float GetCy() const { return cy_; }
  float GetFx() const { return fx_; }
  float GetFy() const { return fy_; }

  CameraIntrinsics GetCameraIntrinsics() const;


 private:

   // Intrinsics
   float fx_ = 1450.317230113;
   float fy_ = 1451.184836113;
   float cx_ = 1244.386581025;
  //  float cx_ = 1480.0;  // TEST TEST
   float cy_ = 1013.145997723;
  //  float cy_ = 1313.145997723; // TEST TEST

   float image_width_ = 2452;
   float image_height_ = 2056;

   float wr_ = 2452.0f / 2056.0f;

   // float width = 1.0f;
   // float height = 1.0f;

   float near_ = 0.1;
   float far_ = 5000;

   float scale_ = 1.0f;

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

  // Initial camera position
   glm::vec3 origin_;

   // TODO (Pavlo): Should be removed probably
   float zoom_;

   void UpdateCameraVectors(bool constrain_pitch = true);

};

#endif  // CV_GL_CAMERA_H_
