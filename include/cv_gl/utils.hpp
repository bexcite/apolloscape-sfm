// Copyright Pavlo 2018
#ifndef CV_GL_UTILS_HPP_
#define CV_GL_UTILS_HPP_

#include <iostream>
#include <boost/filesystem.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include <opencv2/opencv.hpp>
// #include "opencv2/xfeatures2d.hpp"


// #define GLM_ENABLE_EXPERIMENTAL
// #include <glm/gtx/string_cast.hpp>

// namespace fs = boost::filesystem;

struct ImageData {
  std::string filename;
  std::vector<double> coords;
};

template <typename T>
void PrintVec(const std::string& intro, const std::vector<T> vec,
              std::ostream& os = std::cout);

std::ostream& operator<<(std::ostream& os, const ImageData image_data);

// TODO: Copy/Paste - remove ...
std::ostream& operator<<(std::ostream& os, const glm::mat4x3 mat);

std::ostream& operator<<(std::ostream& os, const glm::mat4 mat);

// TODO: mat3 and mat4 should be in one function!!!
std::ostream& operator<<(std::ostream& os, const glm::mat3 mat);

// TODO: This is a bad copy of vec4,3,2 ... 
std::ostream& operator<<(std::ostream& os, const glm::vec4 vec);

std::ostream& operator<<(std::ostream& os, const glm::vec3 vec);

std::ostream& operator<<(std::ostream& os, const glm::vec2 vec);

std::vector<std::string> StringSplit(const std::string& s, const char c);

std::vector<double> StringSplitD(const std::string& s, const char c);

std::vector<ImageData> ReadCameraPoses(boost::filesystem::path file_path);

// template <typename T>
// void PrintVec(const std::string& intro, const std::vector<T> vec,
//               std::ostream& os);

template <typename T>
void PrintVec(const std::string& intro, const std::vector<T> vec,
              std::ostream& os)  {
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



// ======================================================
// ================== SfM Methods =======================
// ======================================================

void KeyPointToPointVec(const std::vector<cv::KeyPoint>& kpoints, std::vector<cv::Point2f>& points);
void KeyPointsToPointVec(const std::vector<cv::KeyPoint>& kpoints1,
                         const std::vector<cv::KeyPoint>& kpoints2,
                         const std::vector<cv::DMatch>& match,
                         std::vector<cv::Point2f>& points1,
                         std::vector<cv::Point2f>& points2);

glm::dmat3 GetRotation(const float x_angle, const float y_angle, const float z_angle);


// ======================================================
// ========== OpenCV Helpers ============================
// ======================================================

void ImShow(const std::string& window_name, const cv::Mat& img, const double scale = 1.0);
void DrawKeypointsWithResize(const cv::Mat& input_img, const std::vector<cv::KeyPoint>& kpoints, cv::Mat& out_img, const double scale = 1.0);
void DrawMatchesWithResize(const cv::Mat& img1,
                           const std::vector<cv::KeyPoint>& kpoints1, 
                           const cv::Mat& img2, 
                           const std::vector<cv::KeyPoint>& kpoints2, 
                           cv::Mat& img_matches, 
                           const double scale = 1.0,
                           const std::vector<cv::DMatch>& match = std::vector<cv::DMatch>());

void ImShowMatchesWithResize(const cv::Mat& img1,
                             const std::vector<cv::KeyPoint>& kpoints1, 
                             const cv::Mat& img2, 
                             const std::vector<cv::KeyPoint>& kpoints2,
                             const std::vector<cv::DMatch>& match,
                             const double scale = 1.0,
                             const int win_x = 0,
                             const int win_y = 10);

std::vector<cv::DMatch> EmptyMatch();





#endif  // CV_GL_UTILS_HPP_
