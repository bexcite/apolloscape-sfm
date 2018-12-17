// Copyright 2018 Pavlo
#ifndef CV_GL_RENDERER_H_
#define CV_GL_RENDERER_H_

#include "cv_gl/camera.h"
#include "cv_gl/dobject.h"

class Renderer {

 public:

  Renderer(const std::shared_ptr<Camera> camera) : camera_(camera) {};

  void Draw(const DObject& object, const bool transparency = false) {

    // Get All Objects
    if (!transparency) {
      object.Draw(camera_->GetViewMatrix(), camera_->GetProjMatrix());  
      return;
    }

    std::cout << "Renderer::Draw with transparency" << std::endl;

    std::shared_ptr<std::vector<DrawableElement> > d_elements = std::make_shared<std::vector<DrawableElement> >();
    object.Draw(camera_->GetViewMatrix(), camera_->GetProjMatrix(), glm::mat4(1.0f), nullptr, d_elements);
    std::cout << "d_elements.size() = " << d_elements->size() << std::endl;;

  };

  void Draw(const DObject* object_ptr, const bool transparency = false) {
    Draw((*object_ptr), transparency);
  };

  void Draw(const std::shared_ptr<DObject>& object_ptr, const bool transparency = false) {
    Draw((*object_ptr), transparency);
  };

 private:
  std::shared_ptr<Camera> camera_;
};

#endif  // CV_GL_RENDERER_H_
