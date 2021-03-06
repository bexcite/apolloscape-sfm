// Copyright 2018 Pavlo
#ifndef CV_GL_DOBJECT_H_
#define CV_GL_DOBJECT_H_

#include <iterator>

#include <glm/glm.hpp>
#include "cv_gl/camera.h"
#include "cv_gl/utils.h"
#include "cv_gl/model.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>

struct DrawableElement {
  std::shared_ptr<Mesh> mesh;
  std::shared_ptr<Shader> shader;
  glm::mat4 view;
  glm::mat4 projection;
  glm::mat4 model;
  float depth;
};


class DObject {

 public:

  // DObject() {
  //   std::cout << "DObject:: (con) - DEFAULT" << std::endl;
  // };

  explicit DObject(const std::shared_ptr<Mesh> mesh = nullptr, const std::string name = "DObject") : mesh_(mesh), shader_(nullptr),
      translation_(glm::vec3(0.0f)),
      scale_(glm::vec3(1.0f)),
      rotation_(glm::mat4(1.0f)),
      no_correction_(false),
      name_(name),
      tag_(0) {
    // std::cout << "DObject:: (con)" << std::endl;
    InitCorrectionMatrix();
  };

  virtual void InitCorrectionMatrix() {
    glm::mat4 model_matrix(1.0f);
    if (!no_correction_) {
      // Correction from OpenGL standard (Z - pointing backwards) to robot coord X - pointing forward)
      model_matrix[0] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
      model_matrix[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
      model_matrix[2] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
      model_matrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
      model_matrix = glm::transpose(model_matrix);
    }
    correction_ = model_matrix;
  };

  void ShaderUseWithMatrix(const std::shared_ptr<Shader> shader, const glm::mat4& view, const glm::mat4& proj,
      const glm::mat4& model) const {
    shader->Use();
    shader->SetMatrix4fv("view", glm::value_ptr(view));
    shader->SetMatrix4fv("projection", glm::value_ptr(proj));
    shader->SetMatrix4fv("model", glm::value_ptr(model));
    PrepareShader(shader);
  }

  virtual void Draw(const glm::mat4& view_matrix, const glm::mat4& proj_matrix,
      const glm::mat4& parent_model_matrix = glm::mat4(1.0f),
      const std::shared_ptr<Shader> default_shader = nullptr,
      std::shared_ptr<std::vector<DrawableElement> > d_elements = nullptr) const {
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

    if (shdr && !d_elements) {
      ShaderUseWithMatrix(shdr, view_matrix, proj_matrix,
          parent_model_matrix * GetModelMatrix());
      // shdr->Use();
      // shdr->SetMatrix4fv("view", glm::value_ptr(view_matrix));
      // shdr->SetMatrix4fv("projection", glm::value_ptr(proj_matrix));
      // shdr->SetMatrix4fv("model", glm::value_ptr(parent_model_matrix * GetModelMatrix()));
      // PrepareShader(shdr);
    }

    // Draw current mesh
    if (mesh_ && !d_elements) {
      mesh_->Draw(shdr);
    }

    if (d_elements && mesh_ && shdr) {
      // std::cout << "Collect d_elements!" << std::endl;
      DrawableElement d_element;
      d_element.mesh = mesh_;
      d_element.shader = shdr;
      d_element.view = view_matrix;
      d_element.model = parent_model_matrix * GetModelMatrix();
      d_element.projection = proj_matrix;
      glm::vec4 el_location = d_element.view * d_element.model * glm::vec4(mesh_->GetCenterPoint(), 1.0f);
      d_element.depth = el_location[2];
      // std::cout << "el_location = " << glm::to_string(el_location) << std::endl;
      // std::cout << "depth = " << d_element.depth << std::endl;
      // std::cout << "transparent = " << mesh_->material.IsTransparent() << std::endl;

      // std::cout << this->name_ << ":mesh = " << mesh_ << std::endl;
      d_elements->push_back(d_element);
    }

    glm::mat4 model_matrix = parent_model_matrix * GetModelMatrix(false);

    // Draw children if any
    for (auto& c : children_) {
      // std::cout << "-=-= DRAW CHILDREN <<<<<<<<<<<< " << std::endl;
      if (c) {
        c->Draw(view_matrix, proj_matrix, model_matrix, shdr, d_elements);
      }
    }

  };

  void AddChild(const std::shared_ptr<DObject>& child) {
    // std::cout << "scale_ = " << glm::to_string(scale_) << std::endl;
    // if (children.empty()) {
    //   std::cout << "children = " << children.size() << std::endl;
    // } else {
    //   std::cout << "children is NULL " << std::endl;
    // }
    
    children_.emplace_back(child);
  }

  void ClearChildren() {
    children_.clear();
  }

  virtual void PrepareShader(const std::shared_ptr<Shader>& shader) const {
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
    "  rotation = " << rotation_; //  std::endl <<
    if (shader_) {
      os << "  shader = " << shader_;
    }
    if (!children_.empty()) {
      os << std::endl << "  children.size = " << children_.size();
    }
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

  glm::vec3 GetTranslation() const {
    return translation_;
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

  glm::mat4 GetRotation() const {
    return rotation_;
  }

  void SetMaterial(const Material& material) {
    mesh_->material = material;
  }

  void SetName(const std::string& name) { name_ = name; }
  std::string GetName() const { return name_; }

  void SetTag(const int tag) { tag_ = tag; }
  int GetTag() const { return tag_; }

  void NoCorrection() {
    no_correction_ = true;
    InitCorrectionMatrix();
  }

  template<typename Func>
  void Apply(const int tag, Func f) {

    // Iterate through children
    for (auto it = children_.begin(); it != children_.end(); ++it) {
      if ((*it)->tag_ == tag) {
        f(*it);
      }
    }
  }

  std::list<std::shared_ptr<DObject> > GetChildren() { return children_; }

  int GetChildrenSize() { return children_.size(); }

  std::shared_ptr<DObject>& GetChild(int idx) {
    assert(idx < children_.size());
    auto it = children_.begin();
    std::advance(it, idx);
    return *it;
  }


 protected:
  std::shared_ptr<Shader> shader_;
  std::shared_ptr<Mesh> mesh_;
  glm::vec3 translation_;
  glm::vec3 scale_;
  glm::mat4 rotation_;
  glm::mat4 correction_;
  std::string name_;

  bool no_correction_;

  std::list<std::shared_ptr<DObject> > children_;

  int tag_;

};



class ColorObject: public DObject {
 public:

  ColorObject(const std::shared_ptr<Mesh> mesh,
              const glm::vec4& color = glm::vec4(1.0f),
              const bool use_vertex_color = false)
      : DObject(mesh, "ColorObject"), color_(color) {
    if (mesh_->GetMeshType() == MeshType::LINES || mesh_->GetMeshType() == MeshType::POINTS) {
      mesh_->material.ambient_color = color;
    } else {
      mesh_->material.diffuse_color = color;
    }
    mesh_->material.use_vertex_color = use_vertex_color;
    if (use_vertex_color) {
      mesh_->material.ambient_color = color;
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
  ImageObject(const std::shared_ptr<Mesh> mesh, bool transparency = false)
      : DObject(mesh, "ImageObject") { 
    mesh_->material.ambient_color = glm::vec4({0.15f, 0.15f, 0.15f, 0.2f});
    mesh_->material.diffuse_color = glm::vec4(0.0f);
    mesh_->material.ambient_transparent = transparency; // enable tranparency
  }

  void SetImage(const cv::Mat mat, bool adjust_aspect_ratio = true) {
    // Set cv::Mat bytes
    Texture texture = Mesh::TextureFromMat(mat, false);
    SetImageImp(texture, adjust_aspect_ratio);
  }

  void SetImage(const std::string& image_path, bool adjust_aspect_ratio = true) {
    // load image and set it into material
    // std::cout << "Set image : " << image_path << std::endl;

    Texture texture = Mesh::TextureFromFile(image_path, "", false, 0.25); // reduce size of the texture
    texture.path = image_path;

    SetImageImp(texture, adjust_aspect_ratio);

    /*
    texture.type = TextureType::AMBIENT;
    // std::cout << "  width = " << texture.width << std::endl;
    // std::cout << "  height = " << texture.height << std::endl;

    if (!mesh_->material.textures.empty()) {
      mesh_->material.textures[0] = texture;
    } else {
      mesh_->material.textures.emplace_back(texture);
    }

    if (adjust_aspect_ratio) {
      this->AddToCorrectionMatrix(glm::scale(glm::mat4(1.0f),
          glm::vec3(static_cast<float>(texture.width) / texture.height, 1.0f, 1.0f)));
    }
    */

  }

  

  void SetImageAlpha(const float alpha) {
    mesh_->material.ambient_color[3] = alpha;
  }

  void SetImageTransparency(const bool transparency) {
    mesh_->material.ambient_transparent = transparency;
  }

private:
  void SetImageImp(Texture texture, bool adjust_aspect_ratio = true) {
    texture.type = TextureType::AMBIENT;

    // std::cout << "  width = " << texture.width << std::endl;
    // std::cout << "  height = " << texture.height << std::endl;

    if (!mesh_->material.textures.empty()) {
      // Delete previous texture
      glDeleteTextures(1, &mesh_->material.textures[0].id);

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
      const glm::mat4& parent_model_matrix = glm::mat4(1.0f),
      const std::shared_ptr<Shader> default_shader = nullptr,
      std::shared_ptr<std::vector<DrawableElement> > d_elements = nullptr) const {

    // std::cout << "ModelObject::Draw" << std::endl;
    DObject::Draw(view_matrix, proj_matrix, parent_model_matrix, default_shader, d_elements);

    std::shared_ptr<Shader> shdr = nullptr;
    if (shader_) {
      shdr = shader_;
    } else if (default_shader) {
      shdr = default_shader;
    }

    if (!shdr) return;

    if (!d_elements) {
      model_->Draw(shdr);
    } else {
      // Get Meshes from the model
      std::vector<std::shared_ptr<Mesh> > meshes = model_->GetMeshes();
      for (const std::shared_ptr<Mesh>& m : meshes) {
        // Create Drawable
        DrawableElement d_element;
        d_element.mesh = m;
        d_element.shader = shdr;
        d_element.view = view_matrix;
        d_element.model = parent_model_matrix * GetModelMatrix();
        d_element.projection = proj_matrix;
        glm::vec4 el_location = d_element.view * d_element.model * glm::vec4(m->GetCenterPoint(), 1.0f);
        d_element.depth = el_location[2];
        // std::cout << "el_location = " << glm::to_string(el_location) << std::endl;
        // std::cout << "depth = " << d_element.depth << std::endl;
        // std::cout << "transparent = " << mesh_->material.IsTransparent() << std::endl;

        // std::cout << this->name_ << ":mesh = " << mesh_ << std::endl;
        d_elements->push_back(d_element);
      }
    }


    // model_->Draw(shdr);

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
