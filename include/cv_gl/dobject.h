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

  // DObject() {
  //   std::cout << "DObject:: (con) - DEFAULT" << std::endl;
  // };

  explicit DObject(const std::shared_ptr<Mesh> mesh = nullptr, const std::string name = "DObject") : mesh_(mesh), shader_(nullptr),
      translation_(glm::vec3(0.0f)),
      scale_(glm::vec3(1.0f)),
      rotation_(glm::mat4(1.0f)),
      name_(name) {
    std::cout << "DObject:: (con)" << std::endl;
    InitCorrectionMatrix();
  };

  virtual void InitCorrectionMatrix() {
    glm::mat4 model_matrix(1.0f);
    // Correction from OpenGL standard (Z - pointing backwards) to robot coord X - pointing forward)
    model_matrix[0] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
    model_matrix[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
    model_matrix[2] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    model_matrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    model_matrix = glm::transpose(model_matrix);
    correction_ = model_matrix;
  };

  virtual void Draw(const glm::mat4& view_matrix, const glm::mat4& proj_matrix,
      const glm::mat4& parent_model_matrix = glm::mat4(1.0f),
      const std::shared_ptr<Shader> default_shader = nullptr) const {
    // std::cout << "DObject::Draw" << std::endl;
    std::shared_ptr<Shader> shdr = nullptr;
    if (shader_) {
      shdr = shader_;
    } else if (default_shader) {
      shdr = default_shader;
    }

    // if (shdr) {
    //   std::cout << "shdr = " << shdr << std::endl;
    // }

    

    if (shdr) {
      shdr->Use();
      shdr->SetMatrix4fv("view", glm::value_ptr(view_matrix));
      shdr->SetMatrix4fv("projection", glm::value_ptr(proj_matrix));
      shdr->SetMatrix4fv("model", glm::value_ptr(parent_model_matrix * GetModelMatrix()));
      PrepareShader(shdr);
    }

    // Draw current mesh
    if (mesh_) {
      mesh_->Draw(shdr);
    }

    glm::mat4 model_matrix = parent_model_matrix * GetModelMatrix(false);

    // Draw children if any
    for (auto& c : children) {
      // std::cout << "-=-= DRAW CHILDREN <<<<<<<<<<<< " << std::endl;
      c->Draw(view_matrix, proj_matrix, model_matrix, shdr);
    }

  };

  void AddChild(const std::shared_ptr<DObject> child) {
    // std::cout << "scale_ = " << glm::to_string(scale_) << std::endl;
    // if (children.empty()) {
    //   std::cout << "children = " << children.size() << std::endl;
    // } else {
    //   std::cout << "children is NULL " << std::endl;
    // }
    
    children.emplace_back(child);
  }

  virtual void PrepareShader(const std::shared_ptr<Shader> shader) const {
    // std::cout << "DObject::PrepareShader" << std::endl;
    // glm::vec4 floor_color = glm::vec4(0.7f, 0.7f, 1.0f, 1.0f);
    // shader->SetVector4fv("color", glm::value_ptr(floor_color));
  };

  void AddToCorrectionMatrix(const glm::mat4 addon_matrix) {
    InitCorrectionMatrix();
    correction_ = addon_matrix * correction_;
  };

  glm::mat4 GetCorrectionMatrix() const {
    return correction_;
  }

  virtual glm::mat4 GetModelMatrix(const bool use_correction = true) const {
    // Create model matrix
    glm::mat4 model_matrix(1.0f);

    if (use_correction) {
      // Correction from OpenGL standard (Z - pointing backwards) to robot coord X - pointing forward)
      // model_matrix[0] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
      // model_matrix[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
      // model_matrix[2] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
      // model_matrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      // model_matrix = glm::transpose(model_matrix);
      model_matrix = GetCorrectionMatrix();
    }
    

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

  void SetRotation(const float x_angle, const float y_angle, const float z_angle ) {
    glm::mat4 rotation(1.0f);    
    rotation = glm::rotate(rotation, z_angle, glm::vec3(0.0f, 0.0f, 1.0f));
    rotation = glm::rotate(rotation, y_angle, glm::vec3(0.0f, 1.0f, 0.0f));
    rotation = glm::rotate(rotation, x_angle, glm::vec3(1.0f, 0.0f, 0.0f));
    rotation_ = rotation;
  }

  void SetMaterial(const Material& material) {
    mesh_->material = material;
  }

  void SetName(const std::string& name) { name_ = name; }
  std::string GetName() { return name_; }



 protected:
  std::shared_ptr<Shader> shader_;
  std::shared_ptr<Mesh> mesh_;
  glm::vec3 translation_;
  glm::vec3 scale_;
  glm::mat4 rotation_;
  glm::mat4 correction_;
  std::string name_;

  std::list<std::shared_ptr<DObject> > children;

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


class ImageObject: public DObject {
public:
  ImageObject(const std::shared_ptr<Mesh> mesh) 
      : DObject(mesh, "ImageObject") { 
  }

  void SetImage(const std::string& image_path, bool adjust_aspect_ratio = true) {
    // load image and set it into material
    std::cout << "Set image : " << image_path << std::endl;

    Texture texture = Mesh::TextureFromFile(image_path, "", false);
    texture.type = TextureType::AMBIENT;
    texture.path = image_path;

    std::cout << "  width = " << texture.width << std::endl;
    std::cout << "  height = " << texture.height << std::endl;

    if (!mesh_->material.textures.empty()) {
      mesh_->material.textures[0] = texture;
    } else {
      mesh_->material.textures.emplace_back(texture);
    }

    if (adjust_aspect_ratio) {
      this->AddToCorrectionMatrix(glm::scale(glm::mat4(1.0f),
          glm::vec3(static_cast<float>(texture.width) / texture.height, 1.0f, 1.0f)));
    }

  }

};



class ModelObject: public DObject {

public:
  ModelObject(Model* model) : model_(model) {}

  virtual void Draw(const glm::mat4& view_matrix, const glm::mat4& proj_matrix,
      const glm::mat4& model_matrix = glm::mat4(1.0f),
      const std::shared_ptr<Shader> default_shader = nullptr) const {

    DObject::Draw(view_matrix, proj_matrix, model_matrix, default_shader);

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
