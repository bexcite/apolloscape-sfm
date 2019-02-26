// Copyright 2018 Pavlo
#ifndef CV_GL_RENDERER_H_
#define CV_GL_RENDERER_H_

#include "cv_gl/camera.h"
#include "cv_gl/dobject.hpp"

class Renderer {

 public:

  Renderer(const std::shared_ptr<Camera> camera) : camera_(camera) {};

  void Draw(const DObject& object, const bool transparency = false) {

    // Get All Objects
    if (!transparency) {
      object.Draw(camera_->GetViewMatrix(), camera_->GetProjMatrix());
      return;
    }

    // std::cout << "Renderer::Draw with transparency" << std::endl;

    // Collect all Drawables 
    std::shared_ptr<std::vector<DrawableElement> > d_elements = std::make_shared<std::vector<DrawableElement> >();

    object.Draw(camera_->GetViewMatrix(), camera_->GetProjMatrix(), glm::mat4(1.0f), nullptr, d_elements);
    // std::cout << "d_elements.size() = " << d_elements->size() << " to draw!" << std::endl;;

    // Sort em by transparency and distant: 
    std::sort(d_elements->begin(), d_elements->end(), [](DrawableElement& a, DrawableElement& b) -> bool {
      bool at = a.mesh->material.IsTransparent();
      bool bt = b.mesh->material.IsTransparent();
      return at == bt ? a.depth < b.depth : at < bt;
    });

    // Draw all elements
    for (auto& de : (*d_elements)) {
      int det = de.mesh->material.IsTransparent();
      // std::cout << "de.depth = " << de.depth << ", t = " << det << std::endl;
      de.shader->Use();
      de.shader->SetMatrix4fv("view", glm::value_ptr(de.view));
      de.shader->SetMatrix4fv("projection", glm::value_ptr(de.projection));
      de.shader->SetMatrix4fv("model", glm::value_ptr(de.model));
      de.mesh->Draw(de.shader);
    }

  };

  void Draw(const DObject* object_ptr, const bool transparency = false) {
    if (object_ptr) {
      Draw((*object_ptr), transparency);
    }
  };

  void Draw(const std::shared_ptr<DObject>& object_ptr, const bool transparency = false) {
    if (object_ptr) {
      Draw((*object_ptr), transparency);
    }
  };

 private:
  std::shared_ptr<Camera> camera_;
};

#endif  // CV_GL_RENDERER_H_
