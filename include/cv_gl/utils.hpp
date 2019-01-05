// Copyright Pavlo 2018
#ifndef CV_GL_UTILS_HPP_
#define CV_GL_UTILS_HPP_

#include <boost/filesystem.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>


// #define GLM_ENABLE_EXPERIMENTAL
// #include <glm/gtx/string_cast.hpp>

namespace fs = boost::filesystem;

struct ImageData {
  std::string filename;
  std::vector<double> coords;
};

template <typename T>
void PrintVec(const std::string& intro, const std::vector<T> vec,
              std::ostream& os = std::cout);



std::ostream& operator<<(std::ostream& os, const ImageData image_data) {
  os << "filename: " << image_data.filename << ", ";
  PrintVec("coords: ", image_data.coords);
  return os;
}

// TODO: Copy/Paste - remove ...
std::ostream& operator<<(std::ostream& os, const glm::mat4x3 mat) {
  int rows = 3;
  int cols = 4;
  os << std::endl;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      os << std::setprecision(3) << std::setw(6) << mat[j][i];
      if (j < cols - 1) {
        os << ", ";
      }
    }
    os << std::endl;
  }
  return os;
}

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

// TODO: mat3 and mat4 should be in one function!!!
std::ostream& operator<<(std::ostream& os, const glm::mat3 mat) {
  int rows = 3;
  int cols = 3;
  os << std::endl;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      os << std::setprecision(3) << std::setw(6) << mat[j][i];
      if (j < cols - 1) {
        os << ", ";
      }
    }
    os << std::endl;
  }
  return os;
}

// TODO: This is a bad copy of vec4,3,2 ... 
std::ostream& operator<<(std::ostream& os, const glm::vec4 vec) {
  for (int j = 0; j < 4; ++j) {
    os << std::setprecision(3) << std::setw(6) << vec[j];
    if (j < 3) {
      os << ", ";
    }
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const glm::vec3 vec) {
  os << std::setprecision(3) << std::setw(6) << vec[0];
  for (int j = 1; j < 3; ++j) {
    os << ", " << std::setprecision(3) << std::setw(6) << vec[j];
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const glm::vec2 vec) {
  os << "{";
  os << std::setprecision(5) << std::setw(6) << vec[0];
  for (int j = 1; j < 2; ++j) {
    os << ", " << std::setprecision(5) << std::setw(6) << vec[j];
  }
  os << "}";
  return os;
}

std::vector<std::string> StringSplit(const std::string& s, const char c) {
    std::vector<std::string> result;
    std::size_t pos = 0;
    std::size_t found = s.find(c);
    while (found != std::string::npos) {
      // std::cout << "found1 = " << s.substr(0, found - pos) << "|\n";
      result.push_back(s.substr(pos, found - pos));
      pos = found + 1;
      found = s.find(c, pos);
    }
    // std::cout << "found2 = " << s.substr(pos + 1) << "|\n";
    result.push_back(s.substr(pos));
    return result;
}


std::vector<double> StringSplitD(const std::string& s, const char c) {
  std::vector<double> result;
  double d;
  std::size_t pos = 0;
  std::size_t found = s.find(c);
  while (found != std::string::npos) {
    // std::cout << "substr1 = '" << s.substr(pos, found - pos) << "'\n";
    d = std::stod(s.substr(pos, found - pos));
    result.push_back(d);
    pos = found + 1;
    found = s.find(c, pos);
  }
  // std::cout << "substr2 = '" << s.substr(pos) << "'\n";
  d = std::stod(s.substr(pos));
  result.push_back(d);
  return result;
}

std::vector<ImageData> ReadCameraPoses(fs::path file_path) {
  std::vector<ImageData> imgs_data;
  if (!fs::is_regular_file(file_path)) {
    return imgs_data;
  }

  //  Read file
  std::ifstream camera_pose_file(file_path.string());
  std::string line;
  std::vector<std::string> splits;
  std::vector<double> pose_splits;
  if (camera_pose_file.is_open()) {
    while (std::getline(camera_pose_file, line)) {
      // std::cout << line << std::endl;
      ImageData img_data;
      splits = StringSplit(line, ' ');
      // PrintVec("splits = ", splits);
      // std::cout << "\n";
      // std::cout << "splits size = " << splits.size() << "\n";
      img_data.filename = splits[0];
      pose_splits = StringSplitD(splits[1], ',');
      // PrintVec("pose_splits = ", pose_splits);
      // std::cout << "\n";
      img_data.coords = pose_splits;
      imgs_data.push_back(img_data);
    }
    camera_pose_file.close();
  }
  return imgs_data;
}

template <typename T>
void PrintVec(const std::string& intro, const std::vector<T> vec,
              std::ostream& os) {
  std::string delim;
  if (!intro.empty()) {
    os << intro;
  }
  os << "[";
  for (auto& x : vec) {
    os << delim << x;
    if (delim.empty()) {
      delim = ", ";
    }
  }
  os << "]\n";
}




#endif  // CV_GL_UTILS_HPP_
