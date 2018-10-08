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

 private:
  std::shared_ptr<Camera> camera_;
};

#endif  // CV_GL_RENDERER_H_
