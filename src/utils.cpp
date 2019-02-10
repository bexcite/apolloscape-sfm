// Copyright Pavlo 2018
#include <chrono>

#include <boost/filesystem.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include <iomanip>

#include "cv_gl/utils.hpp"


// #define GLM_ENABLE_EXPERIMENTAL
// #include <glm/gtx/string_cast.hpp>

namespace fs = boost::filesystem;


std::ostream& operator<<(std::ostream& os, const ImageData image_data) {
  os << "record: '" << image_data.record << "', "
     << "image_dir: " << image_data.image_dir << ", "
     << "camera_num: " << image_data.camera_num << ", "
     << "filename: " << image_data.filename << ", ";
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

std::vector<ImageData> ReadCameraPoses(const fs::path file_path,
                                       const fs::path image_dir,
                                       const std::string& record,
                                       const int camera_num) {
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
      img_data.camera_num = camera_num;
      img_data.record = record;
      img_data.image_dir = image_dir.string();
      imgs_data.push_back(img_data);
    }
    camera_pose_file.close();
  }
  return imgs_data;
}


void KeyPointToPointVec(const std::vector<cv::KeyPoint>& kpoints, std::vector<cv::Point2f>& points) {
  for (size_t i = 0; i < kpoints.size(); ++i) {
    points.push_back(kpoints[i].pt);
  }
}

void KeyPointsToPointVec(const std::vector<cv::KeyPoint>& kpoints1,
                         const std::vector<cv::KeyPoint>& kpoints2,
                         const std::vector<cv::DMatch>& match,
                         std::vector<cv::Point2f>& points1,
                         std::vector<cv::Point2f>& points2) {
  for (size_t i = 0; i < match.size(); ++i) {
    points1.push_back(kpoints1[match[i].queryIdx].pt);
    points2.push_back(kpoints2[match[i].trainIdx].pt);
  }
}


glm::dmat3 GetRotation(const float x_angle, const float y_angle, const float z_angle ) {
    glm::dmat4 rotation(1.0f);
    rotation = glm::rotate(rotation, static_cast<double>(z_angle), glm::dvec3(0.0, 0.0, 1.0));
    rotation = glm::rotate(rotation, static_cast<double>(y_angle), glm::dvec3(0.0, 1.0, 0.0));
    rotation = glm::rotate(rotation, static_cast<double>(x_angle), glm::dvec3(1.0, 0.0, 0.0));
    return glm::dmat3(rotation);
}


bool IsPairInOrder(const ImageData& im_data1, const ImageData& im_data2) {
  // TODO: Optimize it by comparing elements rather than combined strings
  auto im1_stem = boost::filesystem::path(im_data1.filename).stem();
  auto im2_stem = boost::filesystem::path(im_data2.filename).stem();
  std::string cache1_name = im_data1.record + "_"
                          + std::to_string(im_data1.camera_num) + "_"
                          + im1_stem.string();
  std::string cache2_name = im_data2.record + "_"
                          + std::to_string(im_data2.camera_num) + "_"
                          + im2_stem.string();
  // std::cout << "cache1_name = " << cache1_name << std::endl;
  // std::cout << "cache2_name = " << cache2_name << std::endl;
  return cache1_name < cache2_name;
}

std::vector<cv::DMatch> EmptyMatch() {
  return std::vector<cv::DMatch>();
}


void ImShow(const std::string& window_name, const cv::Mat& img, const double scale) {
  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(), scale, scale /*, cv::INTER_AREA*/);
  cv::imshow(window_name, resized_img);
}

void DrawKeypointsWithResize(const cv::Mat& input_img,
                             const std::vector<cv::KeyPoint>& kpoints,
                             cv::Mat& out_img,
                             const double scale) {
  std::vector<cv::KeyPoint> kpoints_scaled(kpoints);
  for (size_t k = 0; k < kpoints_scaled.size(); ++k) {
    kpoints_scaled[k].pt.x = static_cast<int>(kpoints_scaled[k].pt.x * scale);
    kpoints_scaled[k].pt.y = static_cast<int>(kpoints_scaled[k].pt.y * scale);
  }
  cv::Mat img_resized;
  // std::cout << "resize1 = " << scale << std::endl << std::flush;
  using namespace std::chrono;
  auto t1_0 = high_resolution_clock::now();
  cv::resize(input_img, img_resized, cv::Size(), scale, scale, cv::INTER_AREA);
  auto t1_1 = high_resolution_clock::now();
  cv::drawKeypoints(img_resized, kpoints_scaled, out_img);
  auto t1_2 = high_resolution_clock::now();
  std::cout << "\n  >> resize_TIME = " 
            << duration_cast<microseconds>(t1_1-t1_0).count() / 1e+6 
            << std::endl;
  std::cout << "\n  >> draw_TIME = " 
            << duration_cast<microseconds>(t1_2-t1_1).count() / 1e+6 
            << std::endl;
}

void DrawMatchesWithResize(const cv::Mat& img1,
                           const std::vector<cv::KeyPoint>& kpoints1, 
                           const cv::Mat& img2, 
                           const std::vector<cv::KeyPoint>& kpoints2, 
                           cv::Mat& img_matches, 
                           const double scale,
                           const std::vector<cv::DMatch>& match) {
  // assert(kpoints1.size() == kpoints2.size());
  std::vector<cv::KeyPoint> kpoints1_scaled(kpoints1);
  std::vector<cv::KeyPoint> kpoints2_scaled(kpoints2);
  for (size_t k = 0; k < kpoints1_scaled.size(); ++k) {
    kpoints1_scaled[k].pt.x = static_cast<int>(kpoints1_scaled[k].pt.x * scale);
    kpoints1_scaled[k].pt.y = static_cast<int>(kpoints1_scaled[k].pt.y * scale);
  }
  for (size_t k = 0; k < kpoints2_scaled.size(); ++k) {
    kpoints2_scaled[k].pt.x = static_cast<int>(kpoints2_scaled[k].pt.x * scale);
    kpoints2_scaled[k].pt.y = static_cast<int>(kpoints2_scaled[k].pt.y * scale);
  }
  cv::Mat img1_resized, img2_resized;
  cv::resize(img1, img1_resized, cv::Size(), scale, scale);
  cv::resize(img2, img2_resized, cv::Size(), scale, scale);
  if (match.size() == 0 && kpoints1.size() == kpoints2.size()) {
    std::vector<cv::DMatch> matches(kpoints1.size());
    for (size_t i = 0; i < kpoints1.size(); ++i) {
      matches[i].queryIdx = i;
      matches[i].trainIdx = i;
    }
    cv::drawMatches(img1_resized, kpoints1_scaled, img2_resized, kpoints2_scaled, matches, img_matches);
  } else {
    cv::drawMatches(img1_resized, kpoints1_scaled, img2_resized, kpoints2_scaled, match, img_matches);
  }
  
}

void ImShowMatchesWithResize(const cv::Mat& img1,
                             const std::vector<cv::KeyPoint>& kpoints1, 
                             const cv::Mat& img2, 
                             const std::vector<cv::KeyPoint>& kpoints2,
                             const std::vector<cv::DMatch>& match,
                             const double scale,
                             const int win_x,
                             const int win_y) {

  std::vector<cv::KeyPoint> kpoints1m, kpoints2m;
  if (!match.empty()) {
    for (size_t i = 0; i < match.size(); ++i) {
      kpoints1m.push_back(kpoints1[match[i].queryIdx]);
      kpoints2m.push_back(kpoints2[match[i].trainIdx]);
    }
  } else {
    kpoints1m = kpoints1;
    kpoints2m = kpoints2;
  }

  cv::Mat img1_points, img2_points, img_matches;
  DrawKeypointsWithResize(img1, kpoints1m, img1_points, scale);
  DrawKeypointsWithResize(img2, kpoints2m, img2_points, scale);
  ImShow("img1", img1_points);
  cv::moveWindow("img1", win_x, win_y);
  ImShow("img2", img2_points);
  cv::moveWindow("img2", win_x + img1_points.size().width, win_y);

  if (!match.empty()) {
    
    std::vector<cv::DMatch> new_match;
    for (size_t i = 0; i < match.size(); ++i) {
      new_match.push_back(cv::DMatch(i, i, match[i].distance));
    }
    DrawMatchesWithResize(img1, kpoints1m,
                          img2, kpoints2m,
                          img_matches, 
                          scale, new_match);
    ImShow("img_matches", img_matches);
    cv::moveWindow("img_matches", win_x, win_y + img1_points.size().height + 20);
  }

}

glm::vec3 GetGlmColorFromImage(const cv::Mat& img, const cv::KeyPoint& point) {
  cv::Vec3b vec = img.at<cv::Vec3b>(point.pt);
  return glm::vec3(vec[2]/255.0, vec[1]/255.0, vec[0]/255.0);
}

glm::vec3 GetGlmColorFromImage(const cv::Mat& img, const cv::Point2f& pt) {
  cv::Vec3b vec = img.at<cv::Vec3b>(pt);
  return glm::vec3(vec[2]/255.0, vec[1]/255.0, vec[0]/255.0);
}






