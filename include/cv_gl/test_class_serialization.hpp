// Copyright Pavlo 2018
#ifndef CV_GL_TEST_CLASS_SERIALIZATION_H_
#define CV_GL_TEST_CLASS_SERIALIZATION_H_

#include <cereal/cereal.hpp>
#include <opencv2/opencv.hpp>
#include "cv_gl/test_class.h"

template<class Archive>
void save(Archive& ar, const TClass& tclass) {
  ar(tclass.tc[0], tclass.tc[1]);
}

template<class Archive>
void load(Archive& ar, const TClass& tclass) {
  ar(tclass.tc[0], tclass.tc[1]);
}

// == cv::KeyPoint =========================
// template<class Archive>
// void save(Archive& archive, const cv::KeyPoint& keypoint) {
//   archive(keypoint.pt, keypoint.size, keypoint.angle, 
//       keypoint.response, keypoint.octave, keypoint.class_id);
// }
// template<class Archive>
// void load(Archive& archive, cv::KeyPoint& keypoint) {
//   archive(keypoint.pt, keypoint.size, keypoint.angle, 
//       keypoint.response, keypoint.octave, keypoint.class_id);
// }

#endif  //  CV_GL_TEST_CLASS_SERIALIZATION_H_