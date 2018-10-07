// Copyright Pavlo 2018
#ifndef CV_GL_GL_WINDOW_H_
#define CV_GL_GL_WINDOW_H_

#include <string>
#include "cv_gl/camera.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

class GLWindow {
public:
  GLWindow(const std::string& name, const unsigned int width,
      const unsigned int height);
  ~GLWindow();

  bool IsValid();

  bool IsRunning();
  void RunLoop();



  void SetCamera(const std::shared_ptr<Camera> camera);

  float delta_time;

private:

  // prevent copy constructor and copy assignemnt use
  GLWindow(const GLWindow& window) = delete;
  GLWindow& operator=(const GLWindow& window) = delete;

  void ProcessInput();
  void MouseCallback(double xpos, double ypos);

  GLFWwindow *window_;
  std::shared_ptr<Camera> camera_ = nullptr;

  unsigned int width_;
  unsigned int height_;

  float last_time_;
  float frames_time_;
  unsigned int frames_;

  bool first_mouse_ = true;
  float last_x_;
  float last_y_;

  float delta_time_sum_ = 0;

};

#endif  // CV_GL_GL_WINDOW_H_
