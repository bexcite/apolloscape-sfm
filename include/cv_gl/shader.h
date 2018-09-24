// Copyright Pavlo 2018
#ifndef CV_GL_SHADER_H_
#define CV_GL_SHADER_H_

#include <iostream>
#include <string>

class Shader {
  // Shader();
 public:
  Shader();
  ~Shader() { glDeleteProgram(id_); }

  // Disable Copying and Assignment
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  Shader(const std::string& vertex_shader_path,
         const std::string& fragment_shader_path);
  void PrintHello() const;
  void Use() const { glUseProgram(id_); }
  unsigned int GetId() const { return id_; }
  void SetFloat(const std::string &, const float) const;
  void SetVector4f(const std::string &, const float,
    const float, const float, const float) const;
  void SetInt(const std::string &, const int) const;
  void SetMatrix4fv(const std::string&, float*) const;

 private:
  unsigned int id_;
};

#endif  //  CV_GL_SHADER_H_
