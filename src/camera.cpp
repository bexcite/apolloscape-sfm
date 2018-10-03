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
      world_up_(world_up),
      yaw_(yaw),
      pitch_(pitch),
      front_(glm::vec3(0.0f, 0.0f, -1.0f)),
      movement_speed_(kSpeed),
      mouse_sensitivity_(kSensitivity),
      zoom_(kZoom)
     {
  UpdateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix() const {
  return glm::lookAt(position_, position_ + front_, up_);
}

glm::mat4 Camera::GetProjMatrix() const {
  // My matrix
  float fx = 1450.317230113;
  float fy=1451.184836113;
  float cx=1244.386581025;
  float cy=1013.145997723;
  float width = 2452;
  float height = 2056;

  float near = 0.1;
  float far = 100;

  glm::mat4 persp1;
  persp1[0] = glm::vec4(fx / width, 0.0f, 0.0f, 0.0f);
  persp1[1] = glm::vec4(0.0f, fy / height, 0.0f, 0.0f);
  persp1[2] = glm::vec4(-cx / width, -cy / height, near + far, -1.0f);
  persp1[3] = glm::vec4(0.0f, 0.0f, near * far, 0.0f);

  // std::cout << "persp1 = " << persp1 << std::endl;



  // glm::vec4 p_eye = {1.1f, 1.1f, -2.0f, 1.0f};
  //
  // glm::vec4 p_proj = persp1 * p_eye;
  //
  // std::cout << "p_proj_w = " << p_proj << std::endl;
  //
  // p_proj = p_proj / p_proj[3];
  // std::cout << "p_proj = " << p_proj << std::endl;

  glm::mat4 ortho1;
  ortho1[0] = glm::vec4(2.0f, 0.0f, 0.0f, 0.0f);
  ortho1[1] = glm::vec4(0.0f, 2.0f, 0.0f, 0.0f);
  ortho1[2] = glm::vec4(0.0f, 0.0f, - 2.0f / (far - near), 0.0f);
  ortho1[3] = glm::vec4(-1.0f, -1.0f, - (far + near) / (far - near), 1.0f);

  // std::cout << "ortho = " << ortho1 << std::endl;

  glm::mat4 persp_full = ortho1 * persp1;
  // std::cout << "persp_full = " << persp_full << std::endl;
  //
  // glm::vec4 p_ndc;
  // p_ndc = ortho1 * p_proj;
  // p_ndc = p_ndc / p_ndc[3];
  // std::cout << "p_ndc = " << p_ndc << std::endl;

  return persp_full;

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
    if (pitch_ > 89.0f) {
      pitch_ = 89.0f;
    } else if (pitch_ < - 89.0f) {
      pitch_ = -89.0f;
    }
  }

  glm::vec3 front;
  front.x = cos(glm::radians(pitch_)) * cos(glm::radians(yaw_));
  front.y = sin(glm::radians(pitch_));
  front.z = - cos(glm::radians(pitch_)) * sin(glm::radians(yaw_));
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
  pitch_ = glm::degrees(asin(direction.y));
  yaw_ = glm::degrees(atan2(-direction.z, direction.x));
  // std::cout << "CAMERA D: p, y = " << pitch_ << ", " << yaw_ << std::endl;
  UpdateCameraVectors();
}

void Camera::SetPosition(const glm::vec3& position) {
  position_ = position;
}
