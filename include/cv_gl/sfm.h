#ifndef CV_GL_SFM_H_
#define CV_GL_SFM_H_

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <glm/glm.hpp>

#include "cv_gl/utils.hpp"
#include "cv_gl/camera.h"

cv::Mat CalcFundamental(const CameraIntrinsics& intr1, const ImageData& img_data1, const CameraIntrinsics& intr2, const ImageData& img_data2);


void GetLineMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund);

void GetMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund = cv::Mat());



void TriangulatePoints(const CameraIntrinsics& intr1, const ImageData& img_data1, const std::vector<cv::Point2f>& points1,
    const CameraIntrinsics& intr2, const ImageData& img_data2, const std::vector<cv::Point2f>& points2, cv::Mat& points3d);


#endif  // CV_GL_SFM_H_