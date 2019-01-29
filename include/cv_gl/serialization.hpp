// Copyright Pavlo 2018
#ifndef CV_GL_SERIALIZATION_MAT_H_
#define CV_GL_SERIALIZATION_MAT_H_

#include <cereal/cereal.hpp>

#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"

#include "cv_gl/sfm_common.hpp"


// == ImagePair =========================
template<class Archive>
void save(Archive& archive, const ImagePair& ip) {
  archive(ip.first, ip.second);
}
template<class Archive>
void load(Archive& archive, ImagePair& ip) {
  archive(ip.first, ip.second);
}

// == Features =========================
template<class Archive>
void save(Archive& archive, const Features& f) {
  archive(f.keypoints, f.descriptors);
}
template<class Archive>
void load(Archive& archive, Features& f) {
  archive(f.keypoints, f.descriptors);
}



namespace cv {

// == cv::Point ============================
template<class Archive, typename T>
void save(Archive& archive, const cv::Point_<T>& point) {
  archive(point.x, point.y);
}
template<class Archive, typename T>
void load(Archive& archive, cv::Point_<T>& point) {
  archive(point.x, point.y);
}

// == cv::KeyPoint =========================
template<class Archive>
void save(Archive& archive, const cv::KeyPoint& keypoint) {
  archive(keypoint.pt, keypoint.size, keypoint.angle, 
      keypoint.response, keypoint.octave, keypoint.class_id);
}
template<class Archive>
void load(Archive& archive, cv::KeyPoint& keypoint) {
  archive(keypoint.pt, keypoint.size, keypoint.angle, 
      keypoint.response, keypoint.octave, keypoint.class_id);
}

// CV_PROP_RW int queryIdx; //!< query descriptor index
// CV_PROP_RW int trainIdx; //!< train descriptor index
// CV_PROP_RW int imgIdx;   //!< train image index
// CV_PROP_RW float distance;

// == cv::DMatch =========================
template<class Archive>
void save(Archive& archive, const cv::DMatch& dm) {
  archive(dm.queryIdx, dm.trainIdx, dm.imgIdx, dm.distance);
}
template<class Archive>
void load(Archive& archive, cv::DMatch& dm) {
  archive(dm.queryIdx, dm.trainIdx, dm.imgIdx, dm.distance);
}



// == cv::Mat =============================
// Taken from https://github.com/patrikhuber/eos/blob/master/include/eos/morphablemodel/io/mat_cerealisation.hpp
template<class Archive>
void save(Archive& ar, const cv::Mat& mat) {
  int rows, cols, type;
  bool continuous;

  rows = mat.rows;
  cols = mat.cols;
  type = mat.type();
  continuous = mat.isContinuous();
  ar (rows, cols, type, continuous);
  if (continuous) {
    const int data_size = rows * cols * static_cast<int>(mat.elemSize());
    std::cout << "save data.size = " << data_size << std::endl;
    auto mat_data = cereal::binary_data(mat.ptr(), data_size);
    ar(mat_data);
  } else {
    const int row_size = cols * static_cast<int>(mat.elemSize());
    for (int i = 0; i < rows; ++i) {
      auto row_data = cereal::binary_data(mat.ptr(i), row_size);
      ar(row_data);
    }
  }
}
template<class Archive>
void load(Archive& ar, cv::Mat& mat) {
  int rows, cols, type;
  bool continuous;

  ar (rows, cols, type, continuous);

  if (continuous) {
      mat.create(rows, cols, type);
      const int data_size = rows * cols * static_cast<int>(mat.elemSize());
      auto mat_data = cereal::binary_data(mat.ptr(), data_size);
      ar(mat_data);
  }
  else {
      mat.create(rows, cols, type);
      const int row_size = cols * static_cast<int>(mat.elemSize());
      for (int i = 0; i < rows; i++) {
          auto row_data = cereal::binary_data(mat.ptr(i), row_size);
          ar(row_data);
      }
  }
}

}


#endif  //  CV_GL_SERIALIZATION_MAT_H_
