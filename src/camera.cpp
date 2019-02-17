// Copyright Pavlo 2017

// #include <glad/glad.h>

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


#include "cv_gl/camera.h"

Camera::Camera(const glm::vec3& position,
    const glm::vec3& world_up,
    float yaw,
    float pitch)
    : position_(position),
      origin_(position),
      world_up_(world_up),
      yaw_(yaw),
      pitch_(pitch),
      front_(glm::vec3(1.0f, 0.0f, 0.0f)),
      movement_speed_(kSpeed),
      mouse_sensitivity_(kSensitivity),
      zoom_(kZoom)
     {

  // TEST
  // cx_ = image_width_ / 2;
  // cy_ = image_height_ / 2;

  UpdateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix() const {
  return glm::lookAt(position_, position_ + front_, up_);
}

glm::mat4 Camera::GetProjMatrix() const {

  // Some info: http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/

  // My matrix

  // float image_width = 2 * cx_;
  // float image_height = 2 * cy_;

  float image_width = image_width_;
  float image_height = image_height_;

  glm::mat4 persp;
  persp[0] = glm::vec4(fx_ / image_width, 0.0f, 0.0f, 0.0f);
  persp[1] = glm::vec4(0.0f, fy_ / image_height, 0.0f, 0.0f);
  // persp[2] = glm::vec4(-cx_ / image_width, -cy_ / image_height, near_ + far_, -1.0f);
  persp[2] = glm::vec4(- (1 - cx_ / image_width), -cy_ / image_height, near_ + far_, -1.0f); // TEST TEST
  persp[3] = glm::vec4(0.0f, 0.0f, near_ * far_, 0.0f);

  // std::cout << "persp1 = " << persp1 << std::endl;

  // glm::vec4 p_eye = {1.1f, 1.1f, -2.0f, 1.0f};
  //
  // glm::vec4 p_proj = persp1 * p_eye;
  //
  // std::cout << "p_proj_w = " << p_proj << std::endl;
  //
  // p_proj = p_proj / p_proj[3];
  // std::cout << "p_proj = " << p_proj << std::endl;

  glm::mat4 ortho;
  ortho[0] = glm::vec4(2.0f, 0.0f, 0.0f, 0.0f);
  ortho[1] = glm::vec4(0.0f, 2.0f, 0.0f, 0.0f);
  ortho[2] = glm::vec4(0.0f, 0.0f, - 2.0f / (far_ - near_), 0.0f);
  ortho[3] = glm::vec4(-1.0f, -1.0f, - (far_ + near_) / (far_ - near_), 1.0f);

  // std::cout << "ortho = " << ortho1 << std::endl;

  glm::mat4 proj_full = ortho * persp;
  // std::cout << "persp_full = " << persp_full << std::endl;
  //
  // glm::vec4 p_ndc;
  // p_ndc = ortho1 * p_proj;
  // p_ndc = p_ndc / p_ndc[3];
  // std::cout << "p_ndc = " << p_ndc << std::endl;

  return proj_full;

}

float Camera::GetZoom() const {
  return zoom_;
}

void Camera::ProcessKeyboard(CameraMovement camera_movement, float delta_time) {
  float delta = movement_speed_ * delta_time;
  if (camera_movement == FORWARD) {
    position_ += front_ * delta;
  } else if (camera_movement == BACKWARD) {
    position_ -= front_ * delta;
  } else if (camera_movement == LEFT) {
    position_ -= right_ * delta;
  } else if (camera_movement == RIGHT) {
    position_ += right_ * delta;
  } else if (camera_movement == MOVE_ORIGIN) {
    // position_ = glm::vec3(-3.0f * scale_, 0.0f * scale_, 1.5f * scale_);
    position_ = origin_;
    // SetDirection(glm::vec3(0.0f * scale_));
    // UpdateCameraVectors();
  } else if (camera_movement == MOVE_TOP) {
    position_ = glm::vec3(0.0f, 0.0f, 15.0f) * scale_;
    SetDirection(glm::vec3(0.0f) * scale_);
    UpdateCameraVectors();
  } else if (camera_movement == MOVE_SIDEWAYS_RIGHT) {
    position_ = glm::vec3(-5.0f, -5.0f, 5.0f) * scale_;
    SetDirection(glm::vec3(0.0f) * scale_);
    UpdateCameraVectors();
  } else if (camera_movement == MOVE_SIDEWAYS_LEFT) {
    position_ = glm::vec3(-5.0f, 5.0f, 5.0f) * scale_;
    SetDirection(glm::vec3(0.0f) * scale_);
    UpdateCameraVectors();
  }
  // std::cout << "CAMERA: p, y = " << pitch_ << ", " << yaw_ << std::endl;
  // std::cout << "CAMERA: position_ = " << glm::to_string(position_) << std::endl;
}

void Camera::ProcessMouseInput(float xoffset, float yoffset,
    bool constrain_pitch) {
  yaw_ -= mouse_sensitivity_ * xoffset;
  pitch_ += mouse_sensitivity_ * yoffset;
  UpdateCameraVectors(constrain_pitch);
}

void Camera::ProcessMouseScroll(float yoffset) {
  zoom_ -= yoffset;
  if (zoom_ < 1.0f) {
    zoom_ = 1.0f;
  } else if (zoom_ > 45.0f) {
    zoom_ = 45.0f;
  }
  // std::cout << "CAMERA: zoom = " << zoom_ << std::endl;
}

void Camera::UpdateCameraVectors(bool constrain_pitch) {
  if (constrain_pitch) {
    if (pitch_ > 89.9f) {
      pitch_ = 89.9f;
    } else if (pitch_ < - 89.9f) {
      pitch_ = -89.9f;
    }
  }

  glm::vec3 front;

  // front.x = cos(glm::radians(pitch_)) * cos(glm::radians(yaw_));
  // front.y = sin(glm::radians(pitch_));
  // front.z = - cos(glm::radians(pitch_)) * sin(glm::radians(yaw_));
  // glm::vec3 front_orig;

  front.x = cos(glm::radians(pitch_)) * cos(glm::radians(yaw_));
  front.y = cos(glm::radians(pitch_)) * sin(glm::radians(yaw_));
  front.z = sin(glm::radians(pitch_));



  front_ = glm::normalize(front);

  right_ = glm::normalize(glm::cross(front_, world_up_));
  up_ = glm::normalize(glm::cross(right_, front_));

  // std::cout << "CAMERA: front_ = " << glm::to_string(front_) << std::endl;
  // std::cout << "CAMERA: p, y = " << pitch_ << ", " << yaw_ << std::endl;
  // std::cout << "CAMERA: position_ = " << glm::to_string(position_) << std::endl;

}

void Camera::SetDirection(const glm::vec3& direction_to) {
  glm::vec3 direction = glm::normalize(direction_to - position_);
  // std::cout << "CAMERA D: direction = " << glm::to_string(direction) << std::endl;
  // pitch_ = glm::degrees(asin(direction.y));
  // yaw_ = glm::degrees(atan2(-direction.z, direction.x));

  pitch_ = glm::degrees(asin(direction.z));
  yaw_ = glm::degrees(atan2(direction.y, direction.x));
  // std::cout << "CAMERA D: p, y = " << pitch_ << ", " << yaw_ << std::endl;
  UpdateCameraVectors();
}

void Camera::SetPosition(const glm::vec3& position) {
  position_ = position;
}

void Camera::SetOrigin(const glm::vec3& origin) {
  origin_ = origin;
  SetPosition(origin_);
}


void Camera::SetRotation(const float x_angle, const float y_angle, const float z_angle) {
  // TODO[PAVLO]: This is the ad-hoc solution that works only on the ZPark rotations angles (sic)
  pitch_ = glm::degrees(y_angle /* + M_PI_2 */);
  yaw_ = glm::degrees(z_angle + M_PI_2);
  UpdateCameraVectors();

  // glm::mat4 rotation(1.0f);
  // rotation = glm::rotate(rotation, z_angle, glm::vec3(0.0f, 0.0f, 1.0f));
  // rotation = glm::rotate(rotation, static_cast<float>(M_PI_2), glm::vec3(0.0f, 1.0f, 0.0f));
}

void Camera::SetRotationPitchYaw(const float pitch_deg, const float yaw_deg) {
  pitch_ = pitch_deg;
  yaw_ = yaw_deg;
  UpdateCameraVectors();
}

void Camera::SetScale(const float scale) {
  scale_ = scale;
}

void Camera::SetIntrinsics(const float fx, const float fy, const float cx,
    const float cy, const float wr = 1.0f) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  wr_ = wr;
}

void Camera::SetIntrinsics(const CameraIntrinsics& camera_intr) {
  this->SetIntrinsics(camera_intr.fx, camera_intr.fy, camera_intr.cx, camera_intr.cy, camera_intr.wr);
}

void Camera::Print(std::ostream& os) const {
    std::cout << "CAMERA: p, y = " << pitch_ << ", " << yaw_ << std::endl;
    std::cout << "CAMERA: position_ = " << glm::to_string(position_) << std::endl;
}

CameraIntrinsics Camera::GetCameraIntrinsics() const {
  CameraIntrinsics intr;
  intr.fx = fx_ / image_width_;
  intr.fy = fy_ / image_height_;
  intr.s = 0.0f ;
  intr.cx = cx_ / image_width_;
  intr.cy = cy_ / image_height_;
  intr.wr = wr_;
  return intr;
}

std::ostream& operator<<(std::ostream& os, const CameraIntrinsics& intr) {
  intr.print(os);
  return os;
}