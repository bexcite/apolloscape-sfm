#ifndef CV_GL_SFM_COMMON_H_
#define CV_GL_SFM_COMMON_H_

#include <vector>
#include <map>
#include <unordered_set>

#include <opencv2/opencv.hpp>

#include "cv_gl/utils.h"
#include "cv_gl/camera.h"
#include "cv_gl/ccomp.hpp"

struct Features {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

struct ImagePair {
  int first;
  int second;
};

inline bool operator==(const ImagePair& ip1, const ImagePair& ip2) {
  return (ip1.first == ip2.first && ip1.second == ip2.second);
}

inline bool operator<(const ImagePair& ip1, const ImagePair& ip2) {
  return (ip1.first != ip2.first ? ip1.first < ip2.first : ip1.second < ip2.second);
}


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
  int component_id;
};

struct Point3DColor {
  Point3DColor() :
      pt{0.0}, color{0.0}, color_br{0.0}, color_bl{0.0}, color_tr{0.0},
      color_tl{0.0}, err{0.0} {}
  glm::vec3 pt;
  glm::vec3 color;
  glm::vec3 color_br;
  glm::vec3 color_bl;
  glm::vec3 color_tr;
  glm::vec3 color_tl;
  double err;
  // int comp_id;
};

typedef std::vector<WorldPoint3D> Map3D;

typedef std::map<int, std::vector<std::pair<int, Point3DColor> > > MapCameras;

double GetCamerasDistance(const CameraInfo& camera_info1, 
                         const CameraInfo& camera_info2);

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
void FilterMatchByLineDistance(const Features& features1, 
                               const CameraInfo camera_info1, 
                               const Features& features2, 
                               const CameraInfo& camera_info2, 
                               Matches& matches, 
                               const double line_dist);

void GetLineImagePoints(const cv::Mat& line, std::vector<cv::Point2f>& line_pts,
                        const double image_width, const double image_height);

void GetLineMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund);

void GetMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund = cv::Mat());

cv::Mat GetProjMatrix(const CameraInfo& camera_info);
cv::Mat GetRotationTranslationTransform(const CameraInfo& camera_info);

void TriangulatePoints(const CameraIntrinsics& intr1, const ImageData& img_data1, const std::vector<cv::Point2f>& points1,
    const CameraIntrinsics& intr2, const ImageData& img_data2, const std::vector<cv::Point2f>& points2, cv::Mat& points3d);

void TriangulatePoints(const CameraInfo& camera_info1, const std::vector<cv::Point2f>& points1,
                       const CameraInfo& camera_info2, const std::vector<cv::Point2f>& points2,
                       cv::Mat& points3d);

std::vector<double> GetReprojectionErrors(
    const std::vector<cv::Point2f>& points,
    const cv::Mat& proj,
    const cv::Mat& points3d);
double GetReprojectionError(
    const Map3D& map,
    const std::vector<CameraInfo>& cameras, 
    const std::vector<Features>& features);
std::vector<double> GetReprojectionErrors(
    const Map3D& map,
    const std::vector<CameraInfo>& cameras, 
    const std::vector<Features>& features);
double GetReprojectionError(const WorldPoint3D& point3d,
                            const std::vector<CameraInfo>& cameras, 
                            const std::vector<Features>& features);
std::vector<double> GetZDistanceFromCamera(const CameraInfo& camera_info,
                                           const cv::Mat& points3d);
void RemoveOutliersByError(Map3D& map,
                           const std::vector<CameraInfo>& cameras,
                           const std::vector<Features>& features,
                           const float percentile);

Map3D ReduceMapByError(const Map3D& map,
                       const std::vector<CameraInfo>& cameras,
                       const std::vector<Features>& features,
                       const float ratio);



int GetNextBestView(const Map3D& map, 
    const std::unordered_set<int>& views, 
    CComponents<std::pair<int, int> >& ccomp,
    const std::vector<Matches>& image_matches,
    const std::map<std::pair<int, int>, int>& matches_index);

int GetNextBestViewByViews(const Map3D& map, 
    const std::unordered_set<int>& todo_views, 
    const std::unordered_set<int>& used_views,
    const std::vector<Matches>& image_matches,
    const std::map<std::pair<int, int>, int>& matches_index);


// TODO: Make CComponents const
void MergeToTheMap(Map3D& map, 
                   const Map3D& local_map, 
                   CComponents<std::pair<int, int> >& ccomp);

void MergeToTheMapImproved(Map3D& map,
                           const Map3D& local_map,
                           CComponents<std::pair<int, int> >& ccomp);

void CombineMapComponents(Map3D& map, const double max_keep_dist);
void MergeAndCombinePoints(Map3D& map,
                           const Map3D& local_map,
                           const double max_keep_dist);


void GetKeyPointColors(const cv::Mat& img, 
                       const cv::KeyPoint& point, 
                       Point3DColor& p3d,
                       const bool add_colors = false,
                       double dangle = 0.0,
                       double image_scale = 1.0);



// ============ Ceres Types / Functions ====
void OptimizeBundle(Map3D& map, const std::vector<CameraInfo>& cameras,
                    const std::vector<Features>& features);



std::ostream& operator<<(std::ostream& os, const ImagePair& ip);
std::ostream& operator<<(std::ostream& os, const WorldPoint3D& wp);

#endif  //  CV_GL_SFM_COMMON_H_