// Copyright Pavlo 2018
#ifndef CV_GL_TEST_CLASS_H_
#define CV_GL_TEST_CLASS_H_

// #include <cereal/cereal.hpp>

#include "cv_gl/utils.hpp"
#include "cv_gl/camera.h"
// #include "cv_gl/serialization_mat.hpp"

#include <opencv2/opencv.hpp>

/*
template<class Archive>
void save(Archive& archive, const cv::KeyPoint& keypoint) {
  // archive(keypoint.pt);
  // archive(keypoint.pt, keypoint.size, keypoint.angle, 
  //     keypoint.response, keypoint.octave, keypoint.class_id);
  archive(keypoint.size);
}
template<class Archive>
void load(Archive& archive, cv::KeyPoint& keypoint) {
  // archive(keypoint.pt);
  // archive(keypoint.pt, keypoint.size, keypoint.angle, 
  //     keypoint.response, keypoint.octave, keypoint.class_id);
  archive(keypoint.size);
}
*/


class TClass {
public:
  TClass() : tc{0,3} {}
  void MakeData();
  int tc[2];
};

class AClass {
public:
  AClass() : ac{0,8} {
    tclass.tc[0] = 18;
    tclass.tc[1] = 28;
    mat.create(3, 4, CV_64F);
  }
  void MakeAData();
  int ac[2];
  TClass tclass;
  cv::Mat mat;
  cv::KeyPoint kp;
  template<class Archive>
  void serialize(Archive& ar) {
    ar(tclass);
    ar(ac[0], ac[1]);
    ar(kp);
    // ar(mat);
  }
};



/*
namespace cv {

// == cv::Mat =============================
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
*/


#endif  //  CV_GL_TEST_CLASS_H_
