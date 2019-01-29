

#include <fstream>
#include <iostream>




// #include "test_class.hpp"
#include "cv_gl/sfm.h"

#include <opencv2/opencv.hpp>

#include <cereal/cereal.hpp>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>

#include "cv_gl/test_class.hpp"
#include "cv_gl/test_class_serialization.hpp"
#include "cv_gl/serialization.hpp"

// template<class Archive>
// void save(Archive& ar, const TClass& tclass) {
//   ar(tclass.tc[0], tclass.tc[1]);
// }

// template<class Archive>
// void load(Archive& ar, const TClass& tclass) {
//   ar(tclass.tc[0], tclass.tc[1]);
// }

// == cv::KeyPoint =========================

// template<>
// void save(cereal::XMLOutputArchive&, const cv::KeyPoint&);
// template<>
// void load(cereal::XMLOutputArchive&, cv::KeyPoint&);

class BClass {
public:
  BClass() : bc{0,8} {
    kp.pt.x = 3.0;
    kp.pt.y = 4.0;
  }
  int bc[2];
  cv::KeyPoint kp;
  // template<class Archive>
  // void serialize(Archive& ar) {
  //   ar(kp);
  //   // ar(mat);
  // }
  // friend class cereal::access;
};

template<class Archive>
void save(Archive& archive, const BClass& bc) {
  archive(bc.kp);
}
template<class Archive>
void load(Archive& archive, BClass& bc) {
  archive(bc.kp);
}


/*
namespace cv {

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

}
*/


int main(int argc, char* argv[]) {
  std::cout << "Test Cereal\n";

  SfM3D sfm;

  int a1 = 20;
  
  cv::Mat mat(3, 4, CV_64F);
  cv::KeyPoint kp;

  BClass bclass;

  TClass tclass;
  AClass aclass;
  // std::cout << "tclass 0 = " << tclass.tc[0]
  //           << ", 1 = " << tclass.tc[1] 
  //           << std::endl;

  

  {
    // std::ofstream farch("tc-out.xml");
    // cereal::XMLOutputArchive archive(farch);

    std::ofstream farch("tc-out.bin", std::ios::binary);
    cereal::BinaryOutputArchive archive(farch);

    archive(tclass);
    archive(aclass);
    archive(a1);
    archive(bclass);
    archive(kp);
    archive(mat);
    archive(sfm);
  }
  // archive(mat);
  // archive(sfm);

  return EXIT_SUCCESS;
}