// Copyright 2018 Pavlo
#ifndef CV_GL_RENDERER_H_
#define CV_GL_RENDERER_H_

#include "cv_gl/camera.h"
#include "cv_gl/dobject.h"

class Renderer {

 public:

  Renderer(const std::shared_ptr<Camera> camera) : camera_(camera) {};

  void Draw(const DObject& object) {
    object.Draw(camera_->GetViewMatrix(), camera_->GetProjMatrix());
  };

  void Draw(const DObject* object_ptr) {
    Draw((*object_ptr));
  };

  void Draw(const std::shared_ptr<DObject>& object_ptr) {
    Draw((*object_ptr));
  };

 private:
  std::shared_ptr<Camera> camera_;
};

#endif  // CV_GL_RENDERER_H_
