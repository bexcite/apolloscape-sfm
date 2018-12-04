// Copyright Pavlo 2018
#ifndef CV_GL_SHADER_H_
#define CV_GL_SHADER_H_

#include <glad/glad.h>

#include <iostream>
#include <string>

class Shader {
  // Shader();
 public:
  Shader();
  ~Shader() { 
    std::cout << "Delete shader: " << (count_--) << std::endl;
    glDeleteProgram(id_); 
  }

  // Disable Copying and Assignment
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  Shader(const std::string& vertex_shader_path,
         const std::string& fragment_shader_path);
  void Use() const { glUseProgram(id_); }
  unsigned int GetId() const { return id_; }
  void SetFloat(const std::string &, const float) const;
  void SetVector4f(const std::string &, const float,
    const float, const float, const float) const;
  void SetVector4fv(const std::string &, const float*) const;
  void SetInt(const std::string &, const int) const;
  void SetMatrix4fv(const std::string&, const float*) const;

  void print(std::ostream& os = std::cout) const;

 private:
  unsigned int id_;
  std::string vertex_shader_path_;
  std::string fragment_shader_path_;
  static int count_;
};



std::ostream& operator<<(std::ostream &os, const Shader& shader);

std::ostream& operator<<(std::ostream &os, const std::shared_ptr<Shader>& shader);

#endif  //  CV_GL_SHADER_H_
