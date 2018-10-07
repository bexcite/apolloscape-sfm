// Copyright Pavlo 2018
#ifndef CV_GL_UTILS_HPP_
#define CV_GL_UTILS_HPP_

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>

// #define GLM_ENABLE_EXPERIMENTAL
// #include <glm/gtx/string_cast.hpp>


std::ostream& operator<<(std::ostream& os, const glm::mat4 mat) {
  os << std::endl;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      os << std::setprecision(3) << std::setw(6) << mat[j][i];
      if (j < 3) {
        os << ", ";
      }
    }
    os << std::endl;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const glm::vec4 vec) {
  for (int j = 0; j < 4; ++j) {
    os << std::setprecision(3) << std::setw(6) << vec[j];
    if (j < 3) {
      os << ", ";
    }
  }
  return os;
}

#endif  // CV_GL_UTILS_HPP_
