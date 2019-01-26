// Copyright Pavlo 2018
#ifndef CV_GL_GL_WINDOW_H_
#define CV_GL_GL_WINDOW_H_

#include <string>
#include <vector>
#include "cv_gl/camera.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

struct ProcessInputFunc {
  unsigned int key;
  // void (*func)();
  std::function<void(float)> func;
};

class GLWindow {
public:
  GLWindow(const std::string& name, const unsigned int width,
      const unsigned int height);
  ~GLWindow();

  bool IsValid();

  bool IsRunning();
  void RunLoop();

  void SetCamera(const std::shared_ptr<Camera> camera);

  template<typename Func>
  void AddProcessInput(const unsigned int key, Func f);

  double GetTime() { return glfwGetTime(); }

  float delta_time;

  // void (*f_ptr)();
  std::vector<ProcessInputFunc> process_input_funcs_;

private:

  // prevent copy constructor and copy assignemnt use
  GLWindow(const GLWindow& window) = delete;
  GLWindow& operator=(const GLWindow& window) = delete;

  void ProcessInput();
  void MouseCallback(double xpos, double ypos);
  void ProcessInputFunctions();

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

  // QUICK FIX for GLFW bug 3,2,1
  // static int macMoved;

};


template<typename Func>
void GLWindow::AddProcessInput(const unsigned int key, Func f) {
  process_input_funcs_.push_back(ProcessInputFunc{.key = key, .func = f});
}


#endif  // CV_GL_GL_WINDOW_H_
