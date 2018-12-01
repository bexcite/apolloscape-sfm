// Copyright 2018 Pavlo
#ifndef CV_GL_DOBJECT_H_
#define CV_GL_DOBJECT_H_

#include <glm/glm.hpp>
#include "cv_gl/camera.h"
#include "cv_gl/utils.hpp"
#include "cv_gl/model.h"

#include <iostream>


class DObject {

 public:

  DObject(const std::shared_ptr<Mesh> mesh = nullptr, const std::string name = "DObject") : mesh_(mesh), shader_(nullptr),
      translation_(glm::vec3(0.0f)),
      scale_(glm::vec3(1.0f)),
      rotation_(glm::mat4(1.0f)),
      name_(name) {
    // std::cout << "DObject:: (con)" << std::endl;
  };

  virtual void Draw(const glm::mat4& view_matrix, const glm::mat4& proj_matrix,
      const std::shared_ptr<Shader> default_shader = nullptr) const {
    // std::cout << "DObject::Draw" << std::endl;
    std::shared_ptr<Shader> shdr = nullptr;
    if (shader_) {
      shdr = shader_;
    } else if (default_shader) {
      shdr = default_shader;
    }

    if (!shdr) {
      std::cout << "DObject::Draw::NOT SHADER" << std::endl;
      return;
    }

    shdr->Use();

    shdr->SetMatrix4fv("view", glm::value_ptr(view_matrix));
    shdr->SetMatrix4fv("projection", glm::value_ptr(proj_matrix));
    shdr->SetMatrix4fv("model", glm::value_ptr(GetModelMatrix()));

    PrepareShader(shdr);

    if (mesh_) {
      mesh_->Draw(shdr);
    }

  };

  virtual void PrepareShader(const std::shared_ptr<Shader> shader) const {
    // std::cout << "DObject::PrepareShader" << std::endl;
    // glm::vec4 floor_color = glm::vec4(0.7f, 0.7f, 1.0f, 1.0f);
    // shader->SetVector4fv("color", glm::value_ptr(floor_color));
  };

  virtual glm::mat4 GetModelMatrix() const {
    // Create model matrix
    glm::mat4 model_matrix;
    model_matrix[0] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
    model_matrix[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
    model_matrix[2] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    model_matrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    model_matrix = glm::transpose(model_matrix);

    glm::mat4 all_model(1.0f);
    all_model = glm::translate(all_model, translation_);
    all_model = all_model * rotation_;
    all_model = glm::scale(all_model, scale_);
    all_model = all_model * model_matrix;

    return all_model;
  }


  virtual void print(std::ostream& os = std::cout) const {
    os << this->name_ << ": " << std::endl << 
    "  trans = " << translation_ << std::endl <<
    "  scale = " << scale_ << std::endl <<
    "  rotation = " << rotation_ << //  std::endl <<
    "  shader = " << shader_;
    if (mesh_) {
      os << std::endl << "  mesh = " << mesh_;
    }
    
  }

  void SetShader(const std::shared_ptr<Shader> shader) {
    shader_ = shader;
  }

  void SetTranslation(const glm::vec3& translation) {
    translation_ = translation;
  }

  void SetScale(const glm::vec3& scale) {
    scale_ = scale;
  }

  void SetRotation(const glm::mat4& rotation) {
    rotation_ = rotation;
  }

  void SetMaterial(const Material& material) {
    mesh_->material = material;
  }

  std::string GetName() { return name_; }


 protected:
  std::shared_ptr<Shader> shader_;
  std::shared_ptr<Mesh> mesh_;
  glm::vec3 translation_;
  glm::vec3 scale_;
  glm::mat4 rotation_;
  std::string name_;

};



class ColorObject: public DObject {
 public:

  ColorObject(const std::shared_ptr<Mesh> mesh,
              const glm::vec4& color = glm::vec4(1.0f))
      : DObject(mesh, "ColorObject"), color_(color) {
    if (mesh_->GetMeshType() == MeshType::LINES) {
      mesh_->material.ambient_color = color;
    } else {
      mesh_->material.diffuse_color = color;
    }
    
  }

  virtual void PrepareShader(const std::shared_ptr<Shader> shader) const {
    // std::cout << "ColorObject::PrepareShader" << std::endl;
    // shader->SetVector4fv("color", glm::value_ptr(color_));

    // Set material
    // shader->SetVector4fv("material.diffuse", glm::value_ptr(mesh_.diffuse_color));
  };

  void SetColor(const glm::vec4& color) {
    color_ = color;
    mesh_->material.diffuse_color = color;
  }

 private:
  glm::vec4 color_;
};



class ModelObject: public DObject {

public:
  ModelObject(Model* model) : model_(model) {}

  virtual void Draw(const glm::mat4& view_matrix, const glm::mat4& proj_matrix,
      const std::shared_ptr<Shader> default_shader = nullptr) const {

    DObject::Draw(view_matrix, proj_matrix, default_shader);

    std::shared_ptr<Shader> shdr = nullptr;
    if (shader_) {
      shdr = shader_;
    } else if (default_shader) {
      shdr = default_shader;
    }
    model_->Draw(shdr);

  }

  virtual void print(std::ostream& os) const {
    DObject::print(os);
    if (model_) {
      os << "  model = " << model_;
    }
  }

private:
  std::shared_ptr<Model> model_;

};



std::ostream& operator<<(std::ostream& os, const DObject& object) {
  object.print(os);
  return os;
}

std::ostream& operator<<(std::ostream& os, const DObject* object) {
  object->print(os);
  return os;
}



#endif  // CV_GL_DOBJECT_H_
