// Copyright 2018 Pavlo
#ifndef CV_GL_DOBJECT_H_
#define CV_GL_DOBJECT_H_

#include <glm/glm.hpp>
#include "cv_gl/camera.h"
#include "cv_gl/utils.hpp"

#include <iostream>


class DObject {

 public:

  DObject(const std::shared_ptr<Mesh> mesh) : mesh_(mesh), shader_(nullptr),
      translation_(glm::vec3(0.0f)),
      scale_(glm::vec3(1.0f)),
      rotation_(glm::mat4(1.0f)) {
    std::cout << "DObject:: (con)" << std::endl;
  };

  virtual void Draw(const glm::mat4& view_matrix, const glm::mat4 proj_matrix,
      const std::shared_ptr<Shader> default_shader = nullptr) const {
    std::cout << "DObject::Draw" << std::endl;
    PrepareShader();
  };

  virtual void PrepareShader() const {
    std::cout << "DObject::PrepareShader" << std::endl;
  };

  void print(std::ostream& os = std::cout) const {
    os << "DObject: trans = " << translation_
       << ", scale = " << scale_ << ", rotation = " << rotation_ << std::endl;
  }

  void SetShader(const std::shared_ptr<Shader> shader) {
    shader_ = shader;
  }

  void SetTranslation(const glm::vec3 translation) {
    translation_ = translation;
  }


 private:
  std::shared_ptr<Shader> shader_;
  std::shared_ptr<Mesh> mesh_;
  glm::vec3 translation_;
  glm::vec3 scale_;
  glm::mat4 rotation_;

};

std::ostream& operator<<(std::ostream& os, const DObject& object) {
  object.print(os);
  return os;
}

#endif  // CV_GL_DOBJECT_H_
