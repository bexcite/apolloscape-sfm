// Copyright Pavlo 2017

#include <glad/glad.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include "cv_gl/shader.h"

int Shader::count_ = 0;

Shader::Shader() {
  std::cout << "Shader: Constructor 0 params" << std::endl;
}

Shader::Shader(const std::string& vertex_shader_path,
    const std::string& fragment_shader_path,
    const std::string& geometry_shader_path) {

  ++count_;

  std::cout << "Shader: Constructor 3 params (" << count_ << ")" << std::endl;
  std::cout << "Vertex s path: " << vertex_shader_path << std::endl;
  std::cout << "Fragment s path: " << fragment_shader_path << std::endl;
  std::cout << "Geometry s path: " << geometry_shader_path << std::endl;

  // store for the debug purposes
  this->vertex_shader_path_ = vertex_shader_path;
  this->fragment_shader_path_ = fragment_shader_path;
  this->geometry_shader_path_ = geometry_shader_path;


  // Read vertex shader
  std::string v_shader_code;
  std::ifstream vsf(vertex_shader_path);
  std::stringstream vs_buffer;
  vs_buffer << vsf.rdbuf();
  v_shader_code = vs_buffer.str();
  // std::cout << "Vertex content: \n" << v_shader_code << std::endl;
  vsf.close();
  const char* v_shader_code_cstr = v_shader_code.c_str();

  // Read fragment shader
  std::string f_shader_code;
  std::ifstream fsf(fragment_shader_path);
  std::stringstream fs_buffer;
  fs_buffer << fsf.rdbuf();
  f_shader_code = fs_buffer.str();
  // std::cout << "Fragment content: \n" << f_shader_code << std::endl;
  fsf.close();
  const char* f_shader_code_cstr = f_shader_code.c_str();

  // Read geometry shader
  std::string g_shader_code;
  if (!geometry_shader_path.empty()) {
    std::ifstream gsf(geometry_shader_path);
    std::stringstream gs_buffer;
    gs_buffer << gsf.rdbuf();
    g_shader_code = gs_buffer.str();
    // std::cout << "Geometry content: \n" << g_shader_code << std::endl;
    gsf.close();
  }
  const char* g_shader_code_cstr = g_shader_code.c_str();

  // Create program

  //  Create vertexShader
  int success;
  char infoLog[512];

  int vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &v_shader_code_cstr, nullptr);
  glCompileShader(vertexShader);
  // check for shader compile errors
  glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED" << infoLog
        << std::endl;
  }

  // Create fragmentShader
  int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &f_shader_code_cstr, nullptr);
  glCompileShader(fragmentShader);
  // check for errors
  glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
    std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_ERROR\n" << infoLog
        << std::endl;
  }

  int geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
  if (!geometry_shader_path.empty()) {
    glShaderSource(geometryShader, 1, &g_shader_code_cstr, nullptr);
    glCompileShader(geometryShader);
    // check for errors
    glGetShaderiv(geometryShader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(geometryShader, 512, nullptr, infoLog);
      std::cout << "ERROR::SHADER::GEOMETRY::COMPILATION_ERROR\n" << infoLog
          << std::endl;
    }
  }

  // link shader into program
  id_ = glCreateProgram();
  glAttachShader(id_, vertexShader);
  if (!geometry_shader_path.empty()) {
    glAttachShader(id_, geometryShader);
  }
  glAttachShader(id_, fragmentShader);
  glLinkProgram(id_);
  glGetProgramiv(id_, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(id_, 512, nullptr, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog
        << std::endl;
  }
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);
  if (!geometry_shader_path.empty()) {
    glDeleteShader(geometryShader);
  }
}

void
Shader::print(std::ostream& os) const {
  os << "Shader: vertex_path = " << this->vertex_shader_path_
     << ", fragment_path = " << this->fragment_shader_path_ 
     << ", geometry_path = " << this->geometry_shader_path_;
}

void
Shader::SetFloat(const std::string& uniform_name, const float value) const {
  int uniform_location = glGetUniformLocation(id_, uniform_name.c_str());
  glUniform1f(uniform_location, value);
}

void
Shader::SetVector4f(const std::string& uniform_name, const float v1,
    const float v2, const float v3, const float v4) const {
  int uniform_location = glGetUniformLocation(id_, uniform_name.c_str());
  glUniform4f(uniform_location, v1, v2, v3, v4);
}

void
Shader::SetVector4fv(const std::string& uniform_name, const float* vec) const {
  int uniform_location = glGetUniformLocation(id_, uniform_name.c_str());
  glUniform4fv(uniform_location, 1, vec);
}


void
Shader::SetInt(const std::string& uniform_name, const int v) const {
  int uniform_location = glGetUniformLocation(id_, uniform_name.c_str());
  glUniform1i(uniform_location, v);
}

void
Shader::SetMatrix4fv(const std::string& uniform_name, const float* matrix) const {
  int uniform_location = glGetUniformLocation(id_, uniform_name.c_str());
  glUniformMatrix4fv(uniform_location, 1, GL_FALSE, matrix);
}

std::ostream&
operator<<(std::ostream& os, const Shader& shader) {
  shader.print(os);
  return os;
}

std::ostream&
operator<<(std::ostream &os, const std::shared_ptr<Shader>& shader) {
  shader->print(os);
  return os;
}

// void Shader::Use() {
//   glUseProgram(id_);
// }
