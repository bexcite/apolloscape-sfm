#ifndef CV_GL_SFM_H_
#define CV_GL_SFM_H_


#include <unordered_set>

#include "cv_gl/sfm_common.h"


// #include <ceres/ceres.h>

#include <opencv2/opencv.hpp>
// #include "opencv2/xfeatures2d.hpp"
#include <glm/glm.hpp>

#include "cv_gl/utils.hpp"
#include "cv_gl/camera.h"
#include "cv_gl/ccomp.hpp"
#include "cv_gl/cache_storage.hpp"

// #include <cereal/cereal.hpp>

// #include "cv_gl/serialization_mat.hpp"

// #include <cereal/details/traits.hpp>

class SfM3D {
public:
  typedef std::pair<int, int> IntPair;
  SfM3D() {}
  explicit SfM3D(std::vector<CameraIntrinsics> camera_intrs)
      : intrinsics_(camera_intrs) {

    p.x = 1.0;
    p.y = 2.1;
    pd.x = 1.0;
    pd.y = 2.1;

    piv.push_back(0);
    piv.push_back(1);
    piv.push_back(2);

    cv::KeyPoint kp;
    kp.pt = pd;
    kp.octave = 4;

    
    kp.pt = pd;
    kps.push_back(kp);
    kp.pt = p;
    kps.push_back(kp);

    mat.create(3, 3, CV_64F);
    dm.queryIdx = 1;
    dm.trainIdx = 2;
    
  }
  void AddImages(const std::vector<ImageData>& camera1_images,
                 const std::vector<ImageData>& camera2_images,
                 const bool make_pairs = true, const int look_back = 5);
  void ExtractFeatures();
  void Print(std::ostream& os = std::cout) const;
  void MatchImageFeatures(const int skip_thresh = 10);

  void InitReconstruction();
  void TriangulatePointsFromViews(const int first_id, 
                                  const int second_id, 
                                  Map3D& map_);
  void OptimizeCurrentMap() { OptimizeMap(map_); }
  void OptimizeMap(Map3D& map);

  void ReconstructAll();
  void ReconstructNextView(const int next_img_id);
  void ReconstructNextViewPair(const int first_id, const int second_id);
  void PrintFinalStats();

  void GetMapPointsVec(std::vector<Point3DColor>& glm_points) const;
  void GetMapCamerasWithPointsVec(
     std::map<int, std::vector<std::pair<int, Point3DColor> > >& map_cameras);
  cv::Mat GetImage(int cam_id) const;
  CameraInfo GetCameraInfo(int cam_id) const;
  cv::KeyPoint GetKeypoint(int cam_id, int point_id) const;

  int FindMaxSizeMatch(const bool within_todo_views = false) const;

  int ImageCount() const;

  bool IsPairInOrder(const int p1, const int p2);

  // TODO: https://www.patrikhuber.ch/blog/6-serialising-opencv-matrices-using-boost-and-cereal
  // https://github.com/patrikhuber/eos/blob/master/include/eos/morphablemodel/io/mat_cerealisation.hpp
  template<class Archive>
  void serialize(Archive& archive) {
    archive (p, pd, piv, kps);
    //archive (p, pd, piv, kps); // , pd /*, kps*/
    // archive(mat);
    // archive(mats);
    archive(ip);
    archive(dm);
    archive(image_features_);
    archive(image_matches_);
  }
private:
  void GenerateAllPairs();

  // Data Initial
  std::vector<CameraIntrinsics> intrinsics_;
  std::vector<ImageData> image_data_;
  std::vector<CameraInfo> cameras_;
  std::vector<cv::Mat> images_;
  std::vector<cv::Mat> image_previews_;

  // Pre-processing & Feature Extraction
  std::vector<Features> image_features_;
  std::vector<ImagePair> image_pairs_;

  // Matching
  CComponents<IntPair> ccomp_;
  std::vector<Matches> image_matches_;
  std::map<IntPair, int> matches_index_;

  // Reconstruction
  std::unordered_set<int> used_views_;
  std::unordered_set<int> todo_views_;
  Map3D map_;

  const double preview_scale_ = 0.3;
  bool use_cache = true;

  // Preprocessing storage
  CacheStorage cache_storage;


  // Test Part
  cv::Point2f p;
  std::vector<int> piv;
  cv::Point2d pd;
  std::vector<cv::KeyPoint> kps;
  std::vector<cv::Mat> mats;
  ImagePair ip;
  cv::DMatch dm;
  cv::Mat mat;
};


std::ostream& operator<<(std::ostream& os, const SfM3D& sfm);


// ============ Ceres Types / Functions ====

// void OptimizeBundle(Map3D& map, const std::vector<CameraInfo>& cameras, const std::vector<Features>& features);

struct SuperCostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

// ==========================================
// ======= Serializations ===================
// ==========================================
// #include "cereal/cereal.hpp"

/*
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

}
*/

/*
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
  // if (continuous) {
  //   const int data_size = rows * cols * static_cast<int>(mat.elemSize());
  //   auto mat_data = cereal::binary_data(mat.ptr(), data_size);
  //   ar(mat_data);
  // } else {
  //   const int row_size = cols * static_cast<int>(mat.elemSize());
  //   for (int i = 0; i < rows; ++i) {
  //     auto row_data = cereal::binary_data(mat.ptr(i), row_size);
  //     ar(row_data);
  //   }
  // }
}
template<class Archive>
void load(Archive& ar, cv::Mat& mat) {
  int rows, cols, type;
  bool continuous;

  ar (rows, cols, type, continuous);

  // if (continuous) {
  //     mat.create(rows, cols, type);
  //     const int data_size = rows * cols * static_cast<int>(mat.elemSize());
  //     auto mat_data = cereal::binary_data(mat.ptr(), data_size);
  //     ar(mat_data);
  // }
  // else {
  //     mat.create(rows, cols, type);
  //     const int row_size = cols * static_cast<int>(mat.elemSize());
  //     for (int i = 0; i < rows; i++) {
  //         auto row_data = cereal::binary_data(mat.ptr(i), row_size);
  //         ar(row_data);
  //     }
  // }
}

// void spec()
// {
//     cv::KeyPoint kp;
//     cv::Mat ttt(3,4,CV_64F);
//     // std::ofstream farch("out.xml");
//     // cereal::XMLOutputArchive archive(farch);
//     // archive(sfm);
//     std::ofstream farch("out.bin", std::ios::binary);
//     cereal::BinaryOutputArchive archive(farch);
//     // archive(sfm);
//     archive(kp);
//   }


// == cv::Mat =============================

*/

#endif  // CV_GL_SFM_H_