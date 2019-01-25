#ifndef CV_GL_SFM_H_
#define CV_GL_SFM_H_

#include <ceres/ceres.h>

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <glm/glm.hpp>

#include "cv_gl/utils.hpp"
#include "cv_gl/camera.h"


struct Features {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

struct ImagePair {
  int first;
  int second;
};

struct Matches {
  ImagePair image_index;
  std::vector<cv::DMatch> match;
};

struct CameraInfo {
  CameraIntrinsics intr;
  glm::dvec3 translation;
  glm::dvec3 rotation_angles;
};

struct WorldPoint3D {
  cv::Point3d pt;
  std::map<int, int> views;
};

typedef std::vector<WorldPoint3D> Map3D;

void CalcFundamental(const CameraInfo& camera_info1, const CameraInfo& camera_info2, cv::Mat& fund);
cv::Mat CalcFundamental(const CameraIntrinsics& intr1, const ImageData& img_data1, const CameraIntrinsics& intr2, const ImageData& img_data2);

void ExtractFeatures(cv::Mat img, Features& features);
void ExtractFeaturesAndCameraInfo(const std::string& image_path,
                                  const ImageData& im_data,
                                  const CameraIntrinsics& intr, 
                                  cv::Mat& img,
                                  Features& features,
                                  CameraInfo& camera_info);
void ComputeLineKeyPointsMatch(const Features& features1, 
                               const CameraInfo camera_info1, 
                               const Features& features2, 
                               const CameraInfo& camera_info2, 
                               Matches& matches);

void GetLineMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund);

void GetMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund = cv::Mat());

cv::Mat GetProjMatrix(const CameraInfo& camera_info);

void TriangulatePoints(const CameraIntrinsics& intr1, const ImageData& img_data1, const std::vector<cv::Point2f>& points1,
    const CameraIntrinsics& intr2, const ImageData& img_data2, const std::vector<cv::Point2f>& points2, cv::Mat& points3d);

void TriangulatePoints(const CameraInfo& camera_info1, const std::vector<cv::Point2f>& points1,
                       const CameraInfo& camera_info2, const std::vector<cv::Point2f>& points2,
                       cv::Mat& points3d);

std::vector<double> GetReprojectionErrors(const std::vector<cv::Point2f>& points, const cv::Mat& proj, const cv::Mat& points3d);

double GetReprojectionError(const Map3D& map, const std::vector<CameraInfo>& cameras, const std::vector<Features>& features);

std::ostream& operator<<(std::ostream& os, const WorldPoint3D& wp);


// ============ Ceres Types / Functions ====

void OptimizeBundle(Map3D& map, const std::vector<CameraInfo>& cameras, const std::vector<Features>& features);

struct SuperCostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};



#endif  // CV_GL_SFM_H_