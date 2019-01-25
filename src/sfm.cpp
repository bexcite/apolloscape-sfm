
#include <algorithm>

#include "cv_gl/utils.hpp"
#include "cv_gl/sfm.h"

#include <boost/filesystem.hpp>


#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

void CalcFundamental(const CameraInfo& camera_info1, const CameraInfo& camera_info2, cv::Mat& fund) {
  glm::dmat3 r1 = GetRotation(camera_info1.rotation_angles[0], camera_info1.rotation_angles[1], camera_info1.rotation_angles[2]);
  // std::cout << "r1 = " << glm::to_string(r1) << std::endl;

  glm::dmat3 r2 = GetRotation(camera_info2.rotation_angles[0], camera_info2.rotation_angles[1], camera_info2.rotation_angles[2]);
  // std::cout << "r2 = " << glm::to_string(r2) << std::endl;

  glm::dmat3 k1 = camera_info1.intr.GetCameraMatrix();
  // std::cout << "k1 = " << glm::to_string(k1) << std::endl;
  glm::dmat3 k2 = camera_info2.intr.GetCameraMatrix();
  // std::cout << "k2 = " << glm::to_string(k2) << std::endl;

  glm::dvec3 t1 = camera_info1.translation;
  // std::cout << "t1 = " << glm::to_string(t1) << std::endl;
  glm::dvec3 t2 = camera_info2.translation;
  // std::cout << "t2 = " << glm::to_string(t2) << std::endl;
  glm::dvec3 b = t2 - t1;
  glm::dmat3x3 sb;
  sb[0] = glm::dvec3(0.0, b[2], -b[1]);
  sb[1] = glm::dvec3(-b[2], 0.0, b[0]);
  sb[2] = glm::dvec3(b[1], -b[0], 0.0);

  glm::dmat3x3 f = glm::transpose(glm::inverse(k1)) * glm::transpose(r1) * sb * r2 * glm::inverse(k2);

  // Transfer to cv::Mat object
  fund.create(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      fund.at<double>(row, col) = f[col][row];
    }
  }


}

cv::Mat CalcFundamental(const CameraIntrinsics& intr1, const ImageData& img_data1, const CameraIntrinsics& intr2, const ImageData& img_data2) {

  CameraInfo camera_info1, camera_info2;

  camera_info1.intr = intr1;
  camera_info1.rotation_angles[0] = img_data1.coords[0];
  camera_info1.rotation_angles[1] = img_data1.coords[1];
  camera_info1.rotation_angles[2] = img_data1.coords[2];
  camera_info1.translation[0] = img_data1.coords[3];
  camera_info1.translation[1] = img_data1.coords[4];
  camera_info1.translation[2] = img_data1.coords[5];

  camera_info2.intr = intr2;
  camera_info2.rotation_angles[0] = img_data2.coords[0];
  camera_info2.rotation_angles[1] = img_data2.coords[1];
  camera_info2.rotation_angles[2] = img_data2.coords[2];
  camera_info2.translation[0] = img_data2.coords[3];
  camera_info2.translation[1] = img_data2.coords[4];
  camera_info2.translation[2] = img_data2.coords[5];

  cv::Mat fund;
  CalcFundamental(camera_info1, camera_info2, fund);

  return fund;


  /*
  // std::cout << "Calc Fundamental.\n";
  // std::cout << "intr1: " << intr1 << std::endl;
  // std::cout << "img_data1: " << img_data1 << std::endl;
  // std::cout << "intr2: " << intr2 << std::endl;
  // std::cout << "img_data2: " << img_data2 << std::endl;

  glm::dmat3 r1 = GetRotation(img_data1.coords[0], img_data1.coords[1], img_data1.coords[2]);
  // std::cout << "r1 = " << glm::to_string(r1) << std::endl;

  glm::dmat3 r2 = GetRotation(img_data2.coords[0], img_data2.coords[1], img_data2.coords[2]);
  // std::cout << "r2 = " << glm::to_string(r2) << std::endl;

  glm::dmat3 k1 = intr1.GetCameraMatrix();
  // std::cout << "k1 = " << glm::to_string(k1) << std::endl;
  glm::dmat3 k2 = intr2.GetCameraMatrix();
  // std::cout << "k2 = " << glm::to_string(k2) << std::endl;

  glm::dvec3 t1(img_data1.coords[3], img_data1.coords[4], img_data1.coords[5]);
  // std::cout << "t1 = " << glm::to_string(t1) << std::endl;
  glm::dvec3 t2(img_data2.coords[3], img_data2.coords[4], img_data2.coords[5]);
  // std::cout << "t2 = " << glm::to_string(t2) << std::endl;
  glm::dvec3 b = t2 - t1;
  glm::dmat3x3 sb;
  sb[0] = glm::dvec3(0.0, b[2], -b[1]);
  sb[1] = glm::dvec3(-b[2], 0.0, b[0]);
  sb[2] = glm::dvec3(b[1], -b[0], 0.0);
  // std::cout << "sb = " << glm::to_string(sb) << std::endl;

  // glm::dmat4x3 t1m(1.0);
  // t1m[3] -= t1;
  // std::cout << "t1m = " << glm::to_string(t1m) << std::endl;

  // glm::dmat4x3 t2m(1.0);
  // t2m[3] -= t2;
  // std::cout << "t2m = " << glm::to_string(t2m) << std::endl;

  // glm::dmat4x3 full1 = k1 * glm::transpose(r1) * t1m;
  // std::cout << "full1 = " << glm::to_string(full1) << std::endl;

  // glm::dvec3 test_point(100.0, 100.0, 1.0);
  // test_point = r1 * test_point + t1;
  // std::cout << "test_point = " << glm::to_string(test_point) << std::endl;

  // std::cout << "test_point_px = " << glm::to_string(full1 * glm::dvec4(test_point, 1.0)) << std::endl;

  glm::dmat3x3 f = glm::transpose(glm::inverse(k1)) * glm::transpose(r1) * sb * r2 * glm::inverse(k2);
  // std::cout << "f = " << glm::to_string(f) << std::endl;

  // glm::dvec3 p1(1630.0, 300.0, 1.0);
  // glm::dvec3 p2(2300.0, 315.0, 1.0);
  // double res = glm::dot(p1, f * p2);
  // std::cout << "res = " << res << std::endl;
  
  // Transfer to cv::Mat object
  cv::Mat fm(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      fm.at<double>(row, col) = f[col][row];
    }
  }

  return fm;
  */
}

void GetFeatureExtractionRegion(const cv::Mat& img, cv::Mat& mask) {
  // cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
  // mask.create(img.size(), CV_8UC1);
  mask = cv::Mat::zeros(img.size(), CV_8UC1);
  //mask.zeros
  cv::Point mask_points[1][4];
  int xmin = (105.0 / 2452.0) * img.size().width;
  int ymin = (90.0 / 2056.0) * img.size().height;
  int xmax = (2356.0 / 2452.0) * img.size().width;
  int ymax = (1956.0 / 2056.0) * img.size().height;
  // mask_points[0][0] = cv::Point(105, 90);
  // mask_points[0][1] = cv::Point(2356, 90);
  // mask_points[0][2] = cv::Point(2356, 1956);
  // mask_points[0][3] = cv::Point(105, 1956);
  mask_points[0][0] = cv::Point(xmin, ymin);
  mask_points[0][1] = cv::Point(xmax, ymin);
  mask_points[0][2] = cv::Point(xmax, ymax);
  mask_points[0][3] = cv::Point(xmin, ymax);

  // std::cout << "p0 = " << mask_points[0][0] << std::endl;
  // std::cout << "p1 = " << mask_points[0][1] << std::endl;
  // std::cout << "p2 = " << mask_points[0][2] << std::endl;
  // std::cout << "p3 = " << mask_points[0][3] << std::endl;

  const cv::Point* mpt[1] = { mask_points[0] };
  int npt[] = { 4 };
  cv::fillPoly(mask, mpt, npt, 1, cv::Scalar(255, 0, 0), cv::LINE_8);
}

void GetLineImagePoints(const cv::Mat& line, std::vector<cv::Point2f>& line_pts, const double image_width, const double image_height) {
    if (abs(line.at<double>(1)) > 10e-9) {
    cv::Point2f p1(0.0, - line.at<double>(2) / line.at<double>(1));
    if (p1.y >= 0.0 && p1.y <= image_height && line_pts.size() < 2) {
      line_pts.push_back(p1);
    }
    cv::Point2f p2(image_width, - (line.at<double>(2) + line.at<double>(0) * image_width) / line.at<double>(1));
    if (p2.y >= 0.0 && p2.y <= image_height && line_pts.size() < 2) {
      line_pts.push_back(p2);
    }
  }
  if (abs(line.at<double>(0)) > 10e-9 && line_pts.size() < 2) {
    cv::Point2f p3(- line.at<double>(2) / line.at<double>(0), 0.0);
    if (p3.x >= 0.0 && p3.x <= image_width && line_pts.size() < 2) {
      line_pts.push_back(p3);
    }
    cv::Point2f p4(- (line.at<double>(2) + line.at<double>(1) * image_height) / line.at<double>(0), image_height);
    if (p4.x >= 0.0 && p4.x <= image_width && line_pts.size() < 2) {
      line_pts.push_back(p4);
    }
  }
  // std::cout << "line_pts.size() = " << line_pts.size() << std::endl;    
  // std::cout << "line_pts[0] = " << line_pts[0] << std::endl;
  // std::cout << "line_pts[1] = " << line_pts[1] << std::endl;
}

void ExtractFeatures(cv::Mat img, Features& features) {
  cv::Mat feature_mask;
  GetFeatureExtractionRegion(img, feature_mask);
  cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
  detector->detectAndCompute(img, feature_mask, features.keypoints, features.descriptors);
  std::cout << "features.keypoints: " << features.keypoints.size() << std::endl;
  std::cout << "features.descriptors: " << features.descriptors.size() << std::endl;
}

void ExtractFeaturesAndCameraInfo(const std::string& image_path,
                                  const ImageData& im_data,
                                  const CameraIntrinsics& intr, 
                                  cv::Mat& img,
                                  Features& features,
                                  CameraInfo& camera_info) {
  boost::filesystem::path full_image_path = image_path / boost::filesystem::path(im_data.filename);
  img = cv::imread(full_image_path.string().c_str());
  ExtractFeatures(img, features);
  camera_info.intr = intr;
  camera_info.rotation_angles[0] = im_data.coords[0];
  camera_info.rotation_angles[1] = im_data.coords[1];
  camera_info.rotation_angles[2] = im_data.coords[2];
  camera_info.translation[0] = im_data.coords[3];
  camera_info.translation[1] = im_data.coords[4];
  camera_info.translation[2] = im_data.coords[5];
}

void ComputeLineKeyPointsMatch(const Features& features1, 
                               const CameraInfo camera_info1, 
                               const Features& features2, 
                               const CameraInfo& camera_info2, 
                               Matches& matches) {

  // == Compute Fundamental Matrix ==
  cv::Mat fund;
  CalcFundamental(camera_info1, camera_info2, fund);
  // std::cout << "Fundamental Matrix: " << fund << std::endl;

  

  // == Look for One point correcpondance
  
  // std::cout << "Create mask ...." << std::endl;
  // cv::Mat mask = cv::Mat::zeros(features1.keypoints.size(), features2.keypoints.size(), CV_8UC1);

  // cv::Mat mask1 = cv::Mat::zeros(points1.size(), points2.size(), CV_8UC1);

  const std::vector<cv::KeyPoint>& points1 = features1.keypoints;
  const std::vector<cv::KeyPoint>& points2 = features2.keypoints;

  const cv::Mat& descriptors1 = features1.descriptors;
  const cv::Mat& descriptors2 = features2.descriptors;

  cv::Mat points1v(points1.size(), 3, CV_64F);
  cv::Mat points2v(3, points2.size(), CV_64F);
  for (int i = 0; i < points1.size(); ++i) {
    points1v.at<double>(i, 0) = points1[i].pt.x;
    points1v.at<double>(i, 1) = points1[i].pt.y;
    points1v.at<double>(i, 2) = 1.0;
  }
  for (int i = 0; i < points2.size(); ++i) {
    points2v.at<double>(0, i) = points2[i].pt.x;
    points2v.at<double>(1, i) = points2[i].pt.y;
    points2v.at<double>(2, i) = 1.0;
  }

  cv::Mat kp_lines21 = fund * points2v;
  // std::cout << "points1v = " << points1v.rows << ", " << points1v.cols << std::endl;
  // std::cout << "points2v = " << points2v.rows << ", " << points2v.cols << std::endl;
  // std::cout << "kp_lines21 = " << kp_lines21.size() << std::endl;

  // TODO: Divide by sqrt(a^2 + b^2) - ?
  cv::Mat mask = cv::abs(points1v * kp_lines21);
  cv::threshold(mask, mask, 0.01, 1, cv::THRESH_BINARY_INV);
  // std::cout << "mask = " << mask.rows << ", " << mask.cols << std::endl;


  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  std::vector<std::vector<cv::DMatch> > knnMatches;
  // std::cout << "knnMatch ..." << std::endl;
  cv::Mat mask_int;
  mask.convertTo(mask_int, CV_8UC1);
  matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2, mask_int);

  // std::vector<cv::DMatch> good_matches;
  // Filter matches: Lowe's test
  const float ratio_thresh = 0.5f; //0.6
  for (int m = 0; m < knnMatches.size(); ++m) {
    if (knnMatches[m].size() < 2) continue; // no match for the points
    if (knnMatches[m][0].distance < ratio_thresh * knnMatches[m][1].distance) {
      matches.match.push_back(knnMatches[m][0]);
      // good_matches.push_back(cv::DMatch(k, knnMatches[0][0].trainIdx, knnMatches[0][0].distance));
      // std::cout << m << " SQUARE !!!! " << (knnMatches[m][0].distance / knnMatches[m][1].distance)
      // << " (" << knnMatches[m][0].queryIdx << " : " << knnMatches[m][0].trainIdx << ")" << std::endl;
      // cv::drawMarker(img2d,
      //               cv::Point2f(points2[knnMatches[0][0].trainIdx].pt.x,
      //                           points2[knnMatches[0][0].trainIdx].pt.y),
      //               cv::Scalar(255.0, 100.0, 255.0),
      //               cv::MARKER_SQUARE, 35, 8);
    }
  }

  std::cout << "lgood_matches.size = " << matches.match.size() << std::endl;
}



void GetLineMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund) {
  cv::Mat feature_mask;
  GetFeatureExtractionRegion(img1, feature_mask);

  // Step 1:: Detect
  int minHessian = 600;
  // cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
  // cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
  // cv::Ptr<cv::ORB> detector = cv::ORB::create(10000);
  cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();

  std::vector<cv::KeyPoint> points1, points2;
  cv::Mat descriptors1, descriptors2;
  detector->detectAndCompute(img1, feature_mask, points1, descriptors1);
  detector->detectAndCompute(img2, feature_mask, points2, descriptors2);

  std::cout << "lpoints1: " << points1.size() << std::endl;
  std::cout << "lpoints2: " << points2.size() << std::endl;

  std::cout << "Descriptor1: " << descriptors1.size() << std::endl;
  std::cout << "Descriptor2: " << descriptors2.size() << std::endl;

  /*
  // == Find Epipoles
  cv::Mat u, w, vt;
  cv::SVD::compute(fund, w, u, vt);
  std::cout << "Fund Matrix Components:\n";
  std::cout << "w = " << w << std::endl;
  std::cout << "u = " << u << std::endl;
  std::cout << "vt = " << vt << std::endl;
  cv::Mat ep2 = vt.row(2).t();
  cv::Mat ep1 = u.col(2);
  std::cout << "ep1 = " << ep1 / ep1.at<double>(2) << std::endl;
  std::cout << "ep1.t() * F = " << ep1.t() * fund << std::endl;
  std::cout << "ep2 = " << ep2 / ep2.at<double>(2) << std::endl;
  std::cout << "F*ep2 = " << fund * ep2 << std::endl;
  */


  // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

  // == Look for One point correcpondance
  std::vector<cv::DMatch> good_matches;
  std::cout << "Create mask ...." << std::endl;
  cv::Mat mask = cv::Mat::zeros(points1.size(), points2.size(), CV_8UC1);

  // cv::Mat mask1 = cv::Mat::zeros(points1.size(), points2.size(), CV_8UC1);

  cv::Mat points1v(points1.size(), 3, CV_64F);
  cv::Mat points2v(3, points2.size(), CV_64F);
  for (int i = 0; i < points1.size(); ++i) {
    points1v.at<double>(i, 0) = points1[i].pt.x;
    points1v.at<double>(i, 1) = points1[i].pt.y;
    points1v.at<double>(i, 2) = 1.0;
  }
  for (int i = 0; i < points2.size(); ++i) {
    points2v.at<double>(0, i) = points2[i].pt.x;
    points2v.at<double>(1, i) = points2[i].pt.y;
    points2v.at<double>(2, i) = 1.0;
  }

  cv::Mat kp_lines21 = fund * points2v;
  std::cout << "points1v = " << points1v.rows << ", " << points1v.cols << std::endl;
  std::cout << "points2v = " << points2v.rows << ", " << points2v.cols << std::endl;
  std::cout << "kp_lines21 = " << kp_lines21.size() << std::endl;

  // TODO: Divide by sqrt(a^2 + b^2) - ?
  cv::Mat mask1 = cv::abs(points1v * kp_lines21);
  cv::threshold(mask1, mask1, 0.01, 1, cv::THRESH_BINARY_INV);
  std::cout << "mask1 = " << mask1.rows << ", " << mask1.cols << std::endl;


  /*  
  int win_x = 0;
  int win_y = 100;
  double win_scale = 0.3;
  // == Show Camera Images (Keypoints) ==
  cv::Mat img1_points, img2_points, img_matches;
  DrawKeypointsWithResize(img1, points1, img1_points, win_scale);
  DrawKeypointsWithResize(img2, points2, img2_points, win_scale);

  // Debug: Show points
  ImShow("img1", img1_points);
  cv::moveWindow("img1", win_x, win_y);
  ImShow("img2", img2_points);
  cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
  // ImShow("mask", feature_mask, win_scale);
  // ImShow("img matches", img_matches);
  // cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
  cv::waitKey();
  */



  /*
  for (int k = 0; k < points1.size(); ++k) {

    // std::vector<cv::KeyPoint> points1i;
    // points1i.push_back(points1[k]);




    cv::Matx31d kp1_1(points1[k].pt.x, points1[k].pt.y, 1.0);
    // std::cout << "kp1_1 = " << kp1_1 << std::endl;

    cv::Mat kp1_1line = fund.t() * cv::Mat(kp1_1);

    // std::cout << "kp1_1line = " << kp1_1line  << std::endl;

    std::vector<cv::Point2f> line_pts;
    GetLineImagePoints(kp1_1line, line_pts, img1.size().width, img1.size().height);

    // Create mask for one points match
    // cv::Mat mask = cv::Mat::zeros(1, points2.size(), CV_8UC1);
    // cv::Mat::zeros

    // == Debug drawing INIT
    // cv::Mat img1d, img2d;
    // img1d = img1.clone();
    // img2d = img2.clone();

    // == Distance to the points2
    // std::vector<double> dists;
    int cnt = 0;
    // std::cout << "vec = ";
    double d2 = sqrt(pow(kp1_1line.at<double>(0), 2) + pow(kp1_1line.at<double>(1), 2));
    // std::cout << "d2 = " << d2  << std::endl;
    for (size_t j = 0; j < points2.size(); ++j) {
      double d1 = abs(kp1_1line.at<double>(0) * points2[j].pt.x + kp1_1line.at<double>(1) * points2[j].pt.y + kp1_1line.at<double>(2));
      double d = d1; ///d2;
      // std::cout << "d = " << d << std::endl;
      
      // dists.push_back(d);
      if (d < 0.01 ) { // 20
        mask.at<uchar>(k, j) = 1;
        ++cnt;
        // cv::drawMarker(img2d, cv::Point2f(points2[j].pt.x, points2[j].pt.y), cv::Scalar(255.0, 100.0, 100.0), cv::MARKER_CROSS, 40, 8);
        // std::cout << j << " ";
      }
    }
    // if (cnt == 0) {
    //   std::cout << "k = " << k << " is zero\n";
    // }

    // std::cout << std::endl;
    // std::sort(dists.begin(), dists.end());
    // std::cout << "cnt = " << cnt << std::endl;
    // std::cout << "mask = " << mask.size() << std::endl;
    

    

    // for (size_t j = 0; j < dists.size(); ++j) {
    //   if (dists[j] < 10) {
    //     // std::cout << "dists[" << j << "] = " << dists[j] << std::endl;
    //     cv::drawMarker(img2d, cv::Point2f(points2[j].pt.x, points2[j].pt.y), cv::Scalar(255.0, 100.0, 100.0), cv::MARKER_CROSS, 40, 8);
    //   }
    // }
    // cv::Mat img1l;


    // cv::drawMarker(img1d, cv::Point2f(points1i[0].pt.x, points1i[0].pt.y), cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 20, 4);
    // cv::line(img2d, line_pts[0], line_pts[1], cv::Scalar(255.0, 0.0, 0.0), 2);


    // cv::Mat descriptors1i = descriptors1.row(k);
    // std::cout << "descriptors1i = " << descriptors1i.size() << std::endl;


    


    // Step 2: Match
    // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    // std::vector<std::vector<cv::DMatch> > knnMatches;
    
    // matcher->knnMatch(descriptors1i, descriptors2, knnMatches, cnt, mask);
    // std::cout << "lknnMatches.size = " << knnMatches.size() << std::endl;

    


    
    for (size_t i = 0; i < std::min(static_cast<int>(knnMatches[0].size()), 5); ++i) {
      // std::cout << "match 0 = " << knnMatches[i][0].distance << ", " << knnMatches[i][0].queryIdx << " = " << knnMatches[i][0].trainIdx << std::endl;
      // std::cout << "match 1 = " << knnMatches[i][1].distance << ", " << knnMatches[i][1].queryIdx << " = " << knnMatches[i][1].trainIdx << std::endl;
      // std::cout << "match.size = " << knnMatches[i].size() << std::endl;
      // if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance) {
      //   good_matches.push_back(knnMatches[i][0]);
      // }
      // std::cout << "match[" << i << "] = " << knnMatches[0][i].distance 
      //           << ", pi = " << knnMatches[0][i].trainIdx
      //           << ", dist = " << dists[knnMatches[0][i].trainIdx] << std::endl;
      cv::drawMarker(img2d,
                    cv::Point2f(points2[knnMatches[0][i].trainIdx].pt.x,
                                points2[knnMatches[0][i].trainIdx].pt.y),
                    cv::Scalar(100.0, 100.0, 255.0),
                    cv::MARKER_CROSS, 25, 4);
      
    }
    */

    /*
    // Filter matches: Lowe's test
    const float ratio_thresh = 0.5f; //0.6
    bool good_match = false;
    if (knnMatches[0][0].distance < ratio_thresh * knnMatches[0][1].distance) {
      good_match = true;
      good_matches.push_back(cv::DMatch(k, knnMatches[0][0].trainIdx, knnMatches[0][0].distance));
      std::cout << k << " SQUARE !!!! " << (knnMatches[0][0].distance / knnMatches[0][1].distance) << std::endl;
      // cv::drawMarker(img2d,
      //               cv::Point2f(points2[knnMatches[0][0].trainIdx].pt.x,
      //                           points2[knnMatches[0][0].trainIdx].pt.y),
      //               cv::Scalar(255.0, 100.0, 255.0),
      //               cv::MARKER_SQUARE, 35, 8);
      }
    */


    /*
    // == Debug Params
    int win_x = 0;
    int win_y = 100;
    double win_scale = 0.3;

    // == Show Camera Images (Keypoints) ==
    cv::Mat img1_points, img2_points, img_matches;
    DrawKeypointsWithResize(img1d, points1i, img1_points, win_scale);
    DrawKeypointsWithResize(img2d, points1i, img2_points, win_scale);

    if (good_match) {
      // Debug: Show points
      ImShow("img1", img1_points);
      cv::moveWindow("img1", win_x, win_y);
      ImShow("img2", img2_points);
      cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
      // ImShow("img matches", img_matches);
      // cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
      cv::waitKey();
    }
    */

  //}


  // Step 2: Match
  // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  std::vector<std::vector<cv::DMatch> > knnMatches;
  std::cout << "knnMatch ..." << std::endl;
  // cv::Mat mask3 = cv::Mat::zeros(points1.size(), points2.size(), CV_8UC1);
  cv::Mat mask2;
  mask1.convertTo(mask2, CV_8UC1);
  matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2, mask2);

  // Filter matches: Lowe's test
  const float ratio_thresh = 0.4f; //0.6
  bool good_match = false;
  for (int m = 0; m < knnMatches.size(); ++m) {
    if (knnMatches[m].size() < 2) continue;
    if (knnMatches[m][0].distance < ratio_thresh * knnMatches[m][1].distance) {
      good_match = true;
      good_matches.push_back(knnMatches[m][0]);
      // good_matches.push_back(cv::DMatch(k, knnMatches[0][0].trainIdx, knnMatches[0][0].distance));
      // std::cout << m << " SQUARE !!!! " << (knnMatches[m][0].distance / knnMatches[m][1].distance)
      // << " (" << knnMatches[m][0].queryIdx << " : " << knnMatches[m][0].trainIdx << ")" << std::endl;
      // cv::drawMarker(img2d,
      //               cv::Point2f(points2[knnMatches[0][0].trainIdx].pt.x,
      //                           points2[knnMatches[0][0].trainIdx].pt.y),
      //               cv::Scalar(255.0, 100.0, 255.0),
      //               cv::MARKER_SQUARE, 35, 8);
    }
  }

  std::cout << "lgood_matches.size = " << good_matches.size() << std::endl;


  for (size_t i = 0; i < good_matches.size(); ++i) {
    // std::cout << i << " = " << good_matches[i].queryIdx << " : " << good_matches[i].trainIdx << std::endl;
    // Add keypoints to output
    keypoints1.push_back(points1[good_matches[i].queryIdx]);
    keypoints2.push_back(points2[good_matches[i].trainIdx]);
  }

}



void GetMatchedSURFKeypoints(const cv::Mat img1, std::vector<cv::KeyPoint>& keypoints1,
    const cv::Mat img2, std::vector<cv::KeyPoint>& keypoints2, const cv::Mat fund) {

  cv::Mat mask;
  GetFeatureExtractionRegion(img1, mask);

  // Step 1:: Detect
  int minHessian = 400;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
  std::vector<cv::KeyPoint> points1, points2;
  cv::Mat descriptors1, descriptors2;
  detector->detectAndCompute(img1, mask, points1, descriptors1);
  detector->detectAndCompute(img2, mask, points2, descriptors2);

  std::cout << "points1: " << points1.size() << std::endl;
  std::cout << "points2: " << points2.size() << std::endl;


  // std::cout << "Descriptor1: " << descriptors1.size() << std::endl;
  // std::cout << "Descriptor2: " << descriptors2.size() << std::endl;

  // Step 2: Match
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  std::vector<std::vector<cv::DMatch> > knnMatches;
  matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2);
  std::cout << "knnMatches.size = " << knnMatches.size() << std::endl;

  // Filter matches: Lowe's test
  const float ratio_thresh = 0.9f; //0.6
  std::vector<cv::DMatch> good_matches;
  for (size_t i = 0; i < knnMatches.size(); ++i) {
    // std::cout << "match 0 = " << knnMatches[i][0].distance << ", " << knnMatches[i][0].queryIdx << " = " << knnMatches[i][0].trainIdx << std::endl;
    // std::cout << "match 1 = " << knnMatches[i][1].distance << ", " << knnMatches[i][1].queryIdx << " = " << knnMatches[i][1].trainIdx << std::endl;
    // std::cout << "match.size = " << knnMatches[i].size() << std::endl;
    if (knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance) {
      good_matches.push_back(knnMatches[i][0]);

      // Add keypoints to output
      // keypoints1.push_back(points1[knnMatches[i][0].queryIdx]);
      // keypoints2.push_back(points2[knnMatches[i][0].trainIdx]);
    }
  }
  std::cout << "good_matches.size = " << good_matches.size() << std::endl;

  std::vector<cv::DMatch> best_matches;
  if (!fund.empty()) {
    // Check fund matrix constraint
    std::cout << "CHECK FUND MATRIX CONSTRAINT!!!\n";
    // Fundamental Matrix Constraint
    for (size_t i = 0; i < good_matches.size(); ++i) {
      cv::DMatch match = good_matches[i];
      cv::Mat p1 = cv::Mat1d({points1[match.queryIdx].pt.x, points1[match.queryIdx].pt.y, 1.0});
      cv::Mat p2 = cv::Mat1d({points2[match.trainIdx].pt.x, points2[match.trainIdx].pt.y, 1.0});
      cv::Mat pres = p1.t() * fund * p2;
      if (pres.at<double>() < 0.01) {
        // For visualization only
        best_matches.push_back(match);
        keypoints1.push_back(points1[match.queryIdx]);
        keypoints2.push_back(points2[match.trainIdx]);
      } else {
        // std::cout << i << ": good_match res (SKIP) = " << pres << std::endl;
      }
    }

  } else {
    for (size_t i = 0; i < good_matches.size(); ++i) {
      // Add keypoints to output
      keypoints1.push_back(points1[good_matches[i].queryIdx]);
      keypoints2.push_back(points2[good_matches[i].trainIdx]);
    }
  }
  

  /*
  // DEBUG
  // Draw matches
  cv::Mat img_matches;
  if (!fund.empty()) {
    std::cout << "Show BEST matches\n";
    cv::drawMatches(img1, points1, img2, points2, best_matches, img_matches);
  } else {
    std::cout << "Show GOOD matches\n";
    cv::drawMatches(img1, points1, img2, points2, good_matches, img_matches);
  }
  cv::Mat matches_resize;
  cv::resize(img_matches, matches_resize, img_matches.size() / 2);
  cv::imshow("img matches", matches_resize);
  cv::waitKey();
  */


}



void TriangulatePoints(const CameraIntrinsics& intr1, const ImageData& img_data1, const std::vector<cv::Point2f>& points1,
    const CameraIntrinsics& intr2, const ImageData& img_data2, const std::vector<cv::Point2f>& points2, cv::Mat& points3d) {

  CameraInfo camera_info1, camera_info2;

  camera_info1.intr = intr1;
  camera_info1.rotation_angles[0] = img_data1.coords[0];
  camera_info1.rotation_angles[1] = img_data1.coords[1];
  camera_info1.rotation_angles[2] = img_data1.coords[2];
  camera_info1.translation[0] = img_data1.coords[3];
  camera_info1.translation[1] = img_data1.coords[4];
  camera_info1.translation[2] = img_data1.coords[5];

  camera_info2.intr = intr2;
  camera_info2.rotation_angles[0] = img_data2.coords[0];
  camera_info2.rotation_angles[1] = img_data2.coords[1];
  camera_info2.rotation_angles[2] = img_data2.coords[2];
  camera_info2.translation[0] = img_data2.coords[3];
  camera_info2.translation[1] = img_data2.coords[4];
  camera_info2.translation[2] = img_data2.coords[5];

  TriangulatePoints(camera_info1, points1, camera_info2, points2, points3d);

  /*
  std::cout << "Triangulate Points\n";

  // Find Proj Matrices
  glm::dmat3 r1 = GetRotation(img_data1.coords[0], img_data1.coords[1], img_data1.coords[2]);
  // std::cout << "r1 = " << glm::to_string(r1) << std::endl;

  glm::dmat3 r2 = GetRotation(img_data2.coords[0], img_data2.coords[1], img_data2.coords[2]);
  // std::cout << "r2 = " << glm::to_string(r2) << std::endl;

  glm::dmat3 k1 = intr1.GetCameraMatrix();
  // std::cout << "k1 = " << glm::to_string(k1) << std::endl;
  glm::dmat3 k2 = intr2.GetCameraMatrix();
  // std::cout << "k2 = " << glm::to_string(k2) << std::endl;

  glm::dvec3 t1(img_data1.coords[3], img_data1.coords[4], img_data1.coords[5]);
  // std::cout << "t1 = " << glm::to_string(t1) << std::endl;
  glm::dvec3 t2(img_data2.coords[3], img_data2.coords[4], img_data2.coords[5]);
  // std::cout << "t2 = " << glm::to_string(t2) << std::endl;

  glm::dmat4x3 proj1(1.0);
  proj1[3] -= t1;
  proj1 = k1 * glm::transpose(r1) * proj1;
  // std::cout << "proj1 = " << glm::to_string(proj1) << std::endl;

  glm::dmat4x3 proj2(1.0);
  proj2[3] -= t2;
  proj2 = k2 * glm::transpose(r2) * proj2;
  // std::cout << "proj2 = " << glm::to_string(proj2) << std::endl;

  cv::Mat mat_proj1(3, 4, CV_64F);
  cv::Mat mat_proj2(3, 4, CV_64F);
  for (size_t col = 0; col < 4; ++col) {
    for (size_t row = 0; row < 3; ++row) {
      mat_proj1.at<double>(row, col) = proj1[col][row];
      mat_proj2.at<double>(row, col) = proj2[col][row];
    }
  }

  // std::cout << "mat_proj1 = " << mat_proj1 << std::endl;
  // std::cout << "mat_proj2 = " << mat_proj2 << std::endl;

  cv::Mat points4dh;
  cv::triangulatePoints(mat_proj1, mat_proj2, points1, points2, points4dh);

  std::cout << "points4dh.size = " << points4dh.size() << std::endl;

  // Normalize projective
  // points4d.row(0) = points4d.row(0) / points4d.row(3);
  // points4d.row(1) = points4d.row(1) / points4d.row(3);
  // points4d.row(2) = points4d.row(2) / points4d.row(3);
  // points4d.row(3) = points4d.row(3) / points4d.row(3);


  cv::convertPointsFromHomogeneous(points4dh.t(), points3d);

  */

  // std::cout << "points3d[0] = " << points3d.row(0) << std::endl;
  // std::cout << "points3d[10] = " << points3d.row(10) << std::endl;
  // std::cout << "points3d[100] = " << points3d.row(100) << std::endl;
  // std::cout << "points3d[200] = " << points3d.row(200) << std::endl;

  /*
  glm::dvec3 b = t2 - t1;
  glm::dmat3x3 sb;
  sb[0] = glm::dvec3(0.0, b[2], -b[1]);
  sb[1] = glm::dvec3(-b[2], 0.0, b[0]);
  sb[2] = glm::dvec3(b[1], -b[0], 0.0);
  // std::cout << "sb = " << glm::to_string(sb) << std::endl;

  glm::dmat4x3 t1m(1.0);
  t1m[3] -= t1;
  // std::cout << "t1m = " << glm::to_string(t1m) << std::endl;

  glm::dmat4x3 t2m(1.0);
  t2m[3] -= t2;
  // std::cout << "t2m = " << glm::to_string(t2m) << std::endl;

  glm::dmat4x3 full1 = k1 * glm::transpose(r1) * t1m;
  // std::cout << "full1 = " << glm::to_string(full1) << std::endl;
  */
}

cv::Mat GetProjMatrix(const CameraInfo& camera_info) {
  glm::dmat3 r = GetRotation(camera_info.rotation_angles[0], camera_info.rotation_angles[1], camera_info.rotation_angles[2]);
  // std::cout << "r1 = " << glm::to_string(r1) << std::endl;

  glm::dmat3 k = camera_info.intr.GetCameraMatrix();
  // std::cout << "k1 = " << glm::to_string(k1) << std::endl;

  glm::dvec3 t = camera_info.translation;
  // std::cout << "t1 = " << glm::to_string(t1) << std::endl;
  
  glm::dmat4x3 proj(1.0);
  proj[3] -= t;
  proj = k * glm::transpose(r) * proj;
  // std::cout << "proj1 = " << glm::to_string(proj1) << std::endl;

  cv::Mat mat_proj(3, 4, CV_64F);
  for (size_t col = 0; col < 4; ++col) {
    for (size_t row = 0; row < 3; ++row) {
      mat_proj.at<double>(row, col) = proj[col][row];
    }
  }
  return mat_proj;
}


void TriangulatePoints(const CameraInfo& camera_info1, const std::vector<cv::Point2f>& points1,
                       const CameraInfo& camera_info2, const std::vector<cv::Point2f>& points2,
                       cv::Mat& points3d) {
  /*
  glm::dmat3 r1 = GetRotation(camera_info1.rotation_angles[0], camera_info1.rotation_angles[1], camera_info1.rotation_angles[2]);
  // std::cout << "r1 = " << glm::to_string(r1) << std::endl;

  glm::dmat3 r2 = GetRotation(camera_info2.rotation_angles[0], camera_info2.rotation_angles[1], camera_info2.rotation_angles[2]);
  // std::cout << "r2 = " << glm::to_string(r2) << std::endl;

  glm::dmat3 k1 = camera_info1.intr.GetCameraMatrix();
  // std::cout << "k1 = " << glm::to_string(k1) << std::endl;
  glm::dmat3 k2 = camera_info2.intr.GetCameraMatrix();
  // std::cout << "k2 = " << glm::to_string(k2) << std::endl;

  glm::dvec3 t1 = camera_info1.translation;
  // std::cout << "t1 = " << glm::to_string(t1) << std::endl;
  glm::dvec3 t2 = camera_info2.translation;
  
  glm::dmat4x3 proj1(1.0);
  proj1[3] -= t1;
  proj1 = k1 * glm::transpose(r1) * proj1;
  // std::cout << "proj1 = " << glm::to_string(proj1) << std::endl;

  glm::dmat4x3 proj2(1.0);
  proj2[3] -= t2;
  proj2 = k2 * glm::transpose(r2) * proj2;
  // std::cout << "proj2 = " << glm::to_string(proj2) << std::endl;

  cv::Mat mat_proj1(3, 4, CV_64F);
  cv::Mat mat_proj2(3, 4, CV_64F);
  for (size_t col = 0; col < 4; ++col) {
    for (size_t row = 0; row < 3; ++row) {
      mat_proj1.at<double>(row, col) = proj1[col][row];
      mat_proj2.at<double>(row, col) = proj2[col][row];
    }
  }
  */

  cv::Mat mat_proj1 = GetProjMatrix(camera_info1);
  cv::Mat mat_proj2 = GetProjMatrix(camera_info2);

  // std::cout << "mat_proj1 = " << mat_proj1 << std::endl;
  // std::cout << "mat_proj2 = " << mat_proj2 << std::endl;

  cv::Mat points4dh;
  cv::triangulatePoints(mat_proj1, mat_proj2, points1, points2, points4dh);

  std::cout << "points4dh.size = " << points4dh.size() << std::endl;

  cv::convertPointsFromHomogeneous(points4dh.t(), points3d);

}

std::vector<double> GetReprojectionErrors(const std::vector<cv::Point2f>& points, const cv::Mat& proj, const cv::Mat& points3d) {

  std::vector<double> errs(points3d.rows);

  cv::Mat points3dh;
  points3d.convertTo(points3dh, CV_64F);
  cv::convertPointsToHomogeneous(points3dh, points3dh);
  std::cout << "points3dh.row(1) = " << points3dh.row(1) << std::endl << std::flush;
  std::cout << "points3dh.size = " << points3dh.rows << ", " << points3d.cols << std::endl << std::flush;
  std::cout << "points3dh.c = " << points3dh.channels() << std::endl << std::flush;
  std::cout << "points3dh.type = " << points3dh.type() << std::endl << std::flush;
  std::cout << "proj.type = " << proj.type() << std::endl << std::flush;


  // cv::Mat points3dhc(points3dh.rows, 4, CV_64F);
  // cv::Mat points3dhc(points3dh);
  // points3dh.convertTo(points3dhc, CV_64F);
  points3dh = points3dh.reshape(1);
  //cv::Mat points3dhc = points3dh.reshape(1); // (points3dh.rows, 4, CV_64F, points3dh.data);
  // points3dh.convertTo(points3dhc, CV_64F);
  std::cout << "points3dhc.row(1) = " << points3dh.row(1) << std::endl << std::flush;
  std::cout << "points3dhc.type = " << points3dh.type() << std::endl << std::flush;
  std::cout << "proj1.size = " << proj.rows << ", " << proj.cols << std::endl << std::flush;
  std::cout << "points3dhc.size = " << points3dh.rows << ", " << points3dh.cols << std::endl << std::flush;
  std::cout << "points3dhc.c = " << points3dh.channels() << std::endl << std::flush;
  cv::Mat ph = proj * points3dh.t();
  // cv::Mat p2h = proj2 * points3d.t();
  cv::convertPointsFromHomogeneous(ph.t(), ph);
  // cv::convertPointsFromHomogeneous(p2h.t(), p2h);
  std::cout << "p1h.size = " << ph.rows << ", " << ph.cols << std::endl << std::flush;
  std::cout << "p1h.c = " << ph.channels() << std::endl << std::flush;
  // std::cout << "p2h.size = " << p2h.size() << std::endl << std::flush;
  // cv::Mat p1, p2;
  // cv::convertPointsFromHomogeneous(p1h, p1);
  // cv::convertPointsFromHomogeneous(p2h, p2);
  // std::cout << "p1.size = " << p1.size() << std::endl;
  // std::cout << "p2h.size = " << p2h.size() << std::endl;
  std::cout << "p1h.row(0) = " << ph.row(0) << std::endl;
  // std::cout << "p2h.row(0) = " << p2h.row(0) << std::endl;


  for (size_t i = 0; i < points.size(); ++i) {
    double dx, dy;
    double err;
    dx = abs(ph.at<double>(i, 0) - points[i].x);
    dy = abs(ph.at<double>(i, 1) - points[i].y);
    err = sqrt(dx * dx + dy * dy);
    errs[i] = err;
    // std::cout << i << " : err = " << std::fixed << std::setprecision(6) << err << std::endl;
    // p1h.at<double>(i, 0) = abs(p1h.at<double>(i, 0) - points1f[i].x);
    // p1h.at<double>(i, 1) = abs(p1h.at<double>(i, 1) - points1f[i].y);
    // p2h.at<double>(i, 0) = abs(p2h.at<double>(i, 0) - points2f[i].x);
    // p2h.at<double>(i, 1) = abs(p2h.at<double>(i, 1) - points2f[i].y);
  }

  return errs;

}

double GetReprojectionError(const Map3D& map, const std::vector<CameraInfo>& cameras, const std::vector<Features>& features) {

  double err = 0.0;

  for (size_t i = 0; i < map.size(); ++i) {
    for (auto& view : map[i].views) {
      cv::Mat proj = GetProjMatrix(cameras[view.first]);
      const cv::Point2f& point = features[view.first].keypoints[view.second].pt;
      cv::Matx41d point3dh(
        map[i].pt.x, map[i].pt.y, map[i].pt.z, 1.0
      );
      cv::Mat pred = proj * cv::Mat(point3dh);
      pred = pred / pred.at<double>(2);
      double dx = pred.at<double>(0) - double(point.x);
      double dy = pred.at<double>(1) - double(point.y);
      // std::cout << "pred = " << pred << std::endl;
      // std::cout << "dx, dy = " << dx << ", " << dy << std::endl;
      err += (dx * dx + dy * dy);
    }
    
  }

  return err;


}

std::ostream& operator<<(std::ostream& os, const WorldPoint3D& wp) {
  os << "(" << wp.pt << ") from ";
  bool first = true;
  for (auto& v : wp.views) {
    if (!first) {
      os << ", ";
    } else {
      first = false;
    }
    os << "[" << v.first << "," << v.second << "]";
  }
  return os;
}

// =====================================================
// ======== Ceres / BundleOptimization =================

struct ReprojectionErrorFunctor {
  ReprojectionErrorFunctor(const CameraInfo& cam_info, const cv::Point2f& img_point)
      : camera_info(cam_info), image_point(img_point) {

    proj = GetProjMatrix(camera_info);
  }

  template<typename T>
  bool operator()(const T* const point3d,
                  T* residuals) const {
    T p[3];
    p[0] = proj.at<double>(0, 0) * point3d[0] +
           proj.at<double>(0, 1) * point3d[1] +
           proj.at<double>(0, 2) * point3d[2] +
           proj.at<double>(0, 3);
    p[1] = proj.at<double>(1, 0) * point3d[0] +
           proj.at<double>(1, 1) * point3d[1] +
           proj.at<double>(1, 2) * point3d[2] +
           proj.at<double>(1, 3);
    p[2] = proj.at<double>(2, 0) * point3d[0] +
           proj.at<double>(2, 1) * point3d[1] +
           proj.at<double>(2, 2) * point3d[2] +
           proj.at<double>(2, 3);
    // std::cout << "p0/p2 = " << p[0] / p[2] << std::endl;
    // std::cout << "p1/p2 = " << p[1] / p[2] << std::endl;
    residuals[0] = p[0] / p[2] - T(image_point.x);
    residuals[1] = p[1] / p[2] - T(image_point.y);
    // std::cout << "res0 = " << residuals[0] << std::endl;
    // std::cout << "res1 = " << residuals[1] << std::endl;
    return true;
  }

  static ceres::CostFunction* Create(const CameraInfo& cam_info, const cv::Point2f& img_point) {
    return (new ceres::AutoDiffCostFunction<ReprojectionErrorFunctor, 2, 3>(
              new ReprojectionErrorFunctor(cam_info, img_point)));
  }

  const CameraInfo& camera_info;
  const cv::Point2f& image_point;
  cv::Mat proj;
};


void OptimizeBundle(Map3D& map, const std::vector<CameraInfo>& cameras, const std::vector<Features>& features) {

  // TEST output
  double R[2];
  ReprojectionErrorFunctor* f = new ReprojectionErrorFunctor(cameras[2], features[2].keypoints[map[0].views[2]].pt);
  ReprojectionErrorFunctor* f2 = new ReprojectionErrorFunctor(cameras[5], features[5].keypoints[map[0].views[5]].pt);
  ReprojectionErrorFunctor* f3 = new ReprojectionErrorFunctor(cameras[5], features[5].keypoints[map[1].views[5]].pt);
  ReprojectionErrorFunctor* f4 = new ReprojectionErrorFunctor(cameras[5], features[5].keypoints[map[3].views[5]].pt);


  std::cout << "Optimize Bundle!\n";

  ceres::Problem problem;

  
  double* points = new double[3 * map.size()];
  for (size_t i = 0; i < map.size(); ++i) {
    points[3*i + 0] = map[i].pt.x;
    points[3*i + 1] = map[i].pt.y;
    points[3*i + 2] = map[i].pt.z;
  }

  for (size_t i = 0; i < map.size(); ++i) {
    for (auto& view : map[i].views) {
      ceres::CostFunction* cost_function = ReprojectionErrorFunctor::Create(
          cameras[view.first],
          features[view.first].keypoints[view.second].pt);
      problem.AddResidualBlock(cost_function,
          NULL,
          &points[3 * i]);
    }
  }

  // DEBUG OUTPUT
  std::cout << "ERRORS BEFORE: \n";
  (*f)(points, R);
  std::cout << "Residuals: " << R[0] << ", " << R[1] << std::endl;
  (*f2)(points, R);
  std::cout << "Residuals2: " << R[0] << ", " << R[1] << std::endl;
  (*f3)(&points[3], R);
  std::cout << "Residuals3: " << R[0] << ", " << R[1] << std::endl;
  (*f4)(&points[9], R);
  std::cout << "Residuals4: " << R[0] << ", " << R[1] << std::endl;
  std::cout << "points[0-2]: " << points[0] << ", " << points[1] << ", " << points[2] << std::endl;

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500;
  options.eta = 1e-2;
  options.max_solver_time_in_seconds = 10;
  options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  if (not (summary.termination_type == ceres::CONVERGENCE)) {
      std::cerr << "Bundle adjustment failed." << std::endl;
      return;
  }

  
  // DEBUG OUTPUT
  std::cout << "ERRORS AFTER: \n";
  (*f)(points, R);
  std::cout << "Residuals: " << R[0] << ", " << R[1] << std::endl;
  (*f2)(points, R);
  std::cout << "Residuals2: " << R[0] << ", " << R[1] << std::endl;
  (*f3)(&points[3], R);
  std::cout << "Residuals3: " << R[0] << ", " << R[1] << std::endl;
  (*f4)(&points[9], R);
  std::cout << "Residuals4: " << R[0] << ", " << R[1] << std::endl;
  std::cout << "points[0-2]: " << points[0] << ", " << points[1] << ", " << points[2] << std::endl;

  

  // === Copy points back ===
  for (size_t i = 0; i < map.size(); ++i) {
    map[i].pt.x = points[3 * i];
    map[i].pt.y = points[3 * i + 1];
    map[i].pt.z = points[3 * i + 2];
  }

  /*
  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;
  // Build the problem.
  ceres::Problem problem;
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<SuperCostFunctor, 1, 1>(new SuperCostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);
  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  */

  delete[] points;

}

