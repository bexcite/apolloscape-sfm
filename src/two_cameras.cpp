#include <iostream>
#include <memory>
#include <iomanip>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

#include "cv_gl/camera.h"
#include "cv_gl/mesh.h"
#include "cv_gl/shader.h"
#include "cv_gl/utils.hpp"

#include "cv_gl/gl_window.h"

#include "cv_gl/renderer.h"
#include "cv_gl/dobject.h"
#include "cv_gl/object_factory.hpp"

// Some opencvusage: https://github.com/opencv/opencv/blob/master/modules/calib3d/src/fundam.cpp

// How to create empty Mat in OpenCV?
//  https://stackoverflow.com/questions/31337397/how-to-create-empty-mat-in-opencv

const int kWindowWidth = 1226/2;
const int kWindowHeight = 1028/2;

const char kApolloDatasetPath[] = "../data/apolloscape";

const char kRoadId[] = "zpark-sample";
const char kRecordId[] = "Record001";

const char kCamera1PoseFile[] = "Camera_1.txt";
const char kCamera2PoseFile[] = "Camera_2.txt";


cv::Mat estimate_homography(const std::vector<glm::vec2>& points1,
    const std::vector<glm::vec2>& points2 ) {
  cv::Mat hom;

  assert(points1.size() == points2.size());
  int n = points1.size();
  cv::Mat a;
  for (int i = 0; i < n; ++i) {
    auto x1 = points1[i];
    auto x2 = points2[i];
    double wi1 = 1.0;
    double wi2 = 1.0;
    cv::Mat row1 = cv::Mat1d({0.0, 0.0, 0.0, -wi2*x1[0], -wi2*x1[1], -wi2*wi1, x2[1]*x1[0], x2[1]*x1[1], x2[1]*wi1}).t();
    cv::Mat row2 = cv::Mat1d({wi2*x1[0], wi2*x1[1], wi2*wi1, 0.0, 0.0, 0.0, -x2[0]*x1[0], -x2[0]*x1[1], -x2[0]*wi1}).t();
    a.push_back(row1);
    a.push_back(row2);
  }

  /*
  cv::Mat row1 = cv::Mat1d({0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}).t(); // trnaspose because of vector
  cv::Matx<double, 1, 9> row2(2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8);
  // cv::Mat row2 = cv::Mat1d({2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8}).t();
  //cv::Mat row3 = cv::Mat1d({3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8}).t();
  double row3[] = {3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8};
  std::cout << "row1 = " << row1 << std::endl;
  std::cout << "row2 = " << row2 << std::endl;
  
  //a.row(0) << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8;
  a.push_back(row1);
  a.push_back(cv::Mat(row2));
  a.push_back(cv::Mat(1, 9, CV_64F, row3));
  */

  std::cout << "a = " << a << std::endl;

  cv::Mat u, w, vt;
  cv::SVD::compute(a, w, u, vt);
  std::cout << "a.u  = " << u << std::endl;
  std::cout << "a.w  = " << w << std::endl;
  std::cout << "s.vt = " << vt << std::endl;

  std::cout << "sol = " << vt.row(8) << std::endl;

  cv::Mat res = a * vt.row(8).t();
  std::cout << "res = " << res << std::endl;

  cv::Mat v1 = vt * vt.row(8).t();
  std::cout << "v1 = " << v1 << std::endl;

  //vt.row(8).reshape(3, 3);
  
  hom = vt.row(8).reshape(1, 3);
  std::cout << "hom.type = " << hom.type() << std::endl;
  // hom = hom.reshape(1, 3);
  std::cout << "hom = " << hom << std::endl;
  // hom.convertTo(hom, CV_64FC1);
  // std::cout << "hom.type = " << hom.type() << std::endl;


  cv::Mat p1 = cv::Mat1d({points1[0][0], points1[0][1], 1.0});
  cv::Mat p2 = hom * p1;
  std::cout << "p1 = " << p1 << std::endl;
  std::cout << "p2 = " << p2 / p2.at<double>(2) << std::endl;

  // cv::Mat b(a);

  
  // Create Mat

  return hom;
}


int main(int argc, char* argv[]) {
  std::cout << "Hello two cameras" << std::endl;

  fs::path camera1_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
      / fs::path("pose") / fs::path(kRecordId) / fs::path(kCamera1PoseFile);
  fs::path camera2_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
      / fs::path("pose") / fs::path(kRecordId) / fs::path(kCamera2PoseFile);
  std::cout << "Camera 1 path: " << camera1_path << std::endl;
  std::cout << "Camera 2 path: " << camera2_path << std::endl;

  fs::path camera1_image_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
      / fs::path("image") / fs::path(kRecordId) / fs::path("Camera_1");
  fs::path camera2_image_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
      / fs::path("image") / fs::path(kRecordId) / fs::path("Camera_2");

  std::vector<ImageData> camera1_poses = ReadCameraPoses(camera1_path);
  std::vector<ImageData> camera2_poses = ReadCameraPoses(camera2_path);

  std::vector<glm::vec2> camera1_points = {
    {1630.0, 300.0},
    {922.0, 386.0},
    {373.0, 1427.0},
    {384.0, 1413.0},
    {972.0, 949.0},
    {1008.0, 892.0},
    {382.0, 813.0},
    {751.0, 1874.0},
    {718.0, 483.0},
    {1060.0, 580.0},
    {1426.0, 587.0},
    {1722.0, 675.0},
    {1637.0, 817.0},
    {1179.0, 368.0}
  };

  std::vector<glm::vec2> camera2_points = {
    {2300.0, 315.0},
    {1519.0, 369.0},
    {1110.0, 1300.0},
    {1115.0, 1288.0},
    {1537.0, 936.0},
    {1557.0, 884.0},
    {985.0, 734.0},
    {1558.0, 1837.0},
    {1310.0, 455.0},
    {1606.0, 572.0},
    {2043.0, 612.0},
    {2387.0, 749.0},
    {2280.0, 898.0},
    {1757.0, 360.0}
  };

  PrintVec("Camera 1 points: ", camera1_points);
  PrintVec("Camera 2 points: ", camera2_points);

  cv::Mat ones = 2 * cv::Mat::eye(3, 3, CV_32F);
  std::cout << "ones = " << ones << std::endl;
  std::cout << "ones.type = " << ones.type() << std::endl;

  cv::Mat a23 = cv::Mat::zeros(2, 3, CV_32F);
  std::cout << "a23 = " << a23 << std::endl;

  

  std::vector<double> vv = {1.1, 0.2, 2.4};
  cv::Mat ac = cv::Mat(vv).reshape(1, 1);
  std::vector<double> vv1 = {3.0, 3.0, 3.0};
  std::vector<double> vv2 = {5.0, 3.0, 7.0};

  // ac.push_back(vv);
  ac.push_back(cv::Mat(vv1).reshape(1, 1));
  ac.push_back(cv::Mat(vv2).reshape(1, 1));
  std::cout << "ac = " << ac << std::endl;
  std::cout << "ac.type = " << ac.type() << std::endl;

  cv::SVD svd(ac);
  std::cout << "SVD: \n";
  std::cout << "u = " << svd.u << std::endl;
  std::cout << "w = " << svd.w << std::endl;
  std::cout << "vt = " << svd.vt << std::endl;

  cv::Mat hom_res = estimate_homography(camera1_points, camera2_points);
  std::cout << "hom_res = " << hom_res << std::endl;


  // Convert points to Point2f
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  for (int i = 0; i < camera1_points.size(); ++i) {
    points1.push_back(cv::Point2f(camera1_points[i][0], camera1_points[i][1]));
    points2.push_back(cv::Point2f(camera2_points[i][0], camera2_points[i][1]));
  }



  cv::Mat hom_res0 = cv::findHomography(points1, points2, cv::RANSAC, 5);
  std::cout << "hom_res0 = " << hom_res0 << std::endl;

  // Check on all points
  std::cout << "Compare points: " << std::endl;
  double rmse1 = 0.0;
  double rmse2 = 0.0;
  std::vector<cv::Point2f> hom_points2;
  std::vector<cv::Point2f> hom0_points2;
  for (int i = 0; i < camera1_points.size(); ++i) {
    cv::Mat p1 = cv::Mat1d({camera1_points[i][0], camera1_points[i][1], 1.0});
    cv::Mat p2 = hom_res * p1;
    cv::Mat p2a = hom_res0 * p1;

    cv::Point2f p2p;
    p2p.x = p2.at<double>(0) / p2.at<double>(2);
    p2p.y = p2.at<double>(1) / p2.at<double>(2);
    hom_points2.push_back(p2p);

    cv::Point2f p2ap;
    p2ap.x = p2a.at<double>(0) / p2a.at<double>(2);
    p2ap.y = p2a.at<double>(1) / p2a.at<double>(2);
    hom0_points2.push_back(p2ap);

    std::cout << camera2_points[i][0] << " == " << p2p.x << " == " << p2ap.x << ", ";
    std::cout << camera2_points[i][1] << " == " << p2p.y << " == " << p2ap.y << "\n";
    
    rmse1 += pow(camera2_points[i][0] - p2p.x, 2) + pow(camera2_points[i][1] - p2p.y, 2);
    rmse2 += pow(camera2_points[i][0] - p2ap.x, 2) + pow(camera2_points[i][1] - p2ap.y, 2);
  }
  std::cout << "rmse1 = " << sqrt(rmse1) << "\n";
  std::cout << "rmse2 = " << sqrt(rmse2) << "\n";

  int p_camera_img = 24;

  ImageData& im_data1 = camera1_poses[24];
  ImageData& im_data2 = camera2_poses[24];

  fs::path image1_path = camera1_image_path / fs::path(im_data1.filename);
  std::cout << "image1_path = " << image1_path << std::endl;

  fs::path image2_path = camera2_image_path / fs::path(im_data2.filename);
  std::cout << "image2_path = " << image2_path << std::endl;

  cv::Mat img1 = cv::imread(image1_path.string().c_str());
  std::cout << "img1 = " << img1.size << std::endl;
  std::cout << "img1.step = " << img1.step << std::endl;
  std::cout << "img1.step[0] = " << img1.step[0] << std::endl;
  std::cout << "img1.step[1] = " << img1.step[1] << std::endl;
  std::cout << "img1.elemSize = " << img1.elemSize() << std::endl;
  glm::vec3 img1_size_vec = glm::vec3(img1.size().width, img1.size().height, 1.0f);
  std::cout << "img1_size_vec = " << glm::to_string(img1_size_vec) << std::endl;

  cv::Mat img2 = cv::imread(image2_path.string().c_str());
  std::cout << "img2 = " << img2.size << std::endl;
  std::cout << "img2.step = " << img2.step << std::endl;
  std::cout << "img2.step[0] = " << img2.step[0] << std::endl;
  std::cout << "img2.step[1] = " << img2.step[1] << std::endl;
  std::cout << "img2.elemSize = " << img2.elemSize() << std::endl;
  glm::vec3 img2_size_vec = glm::vec3(img2.size().width, img2.size().height, 1.0f);
  std::cout << "img2_size_vec = " << glm::to_string(img2_size_vec) << std::endl;

  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(2.0f, 0.0f, 1.0f));
  

  

  

  
  // =========== Show Points
  for (int i = 0; i < camera1_points.size(); ++i) {
    int delta = 10;
    cv::rectangle(img1,
      cv::Point(camera1_points[i][0] - delta, camera1_points[i][1] - delta),
      cv::Point(camera1_points[i][0] + delta, camera1_points[i][1] + delta),
      cv::Scalar(0, 0, 255),
      5);
  }

  for (int i = 0; i < camera2_points.size(); ++i) {
    int delta = 10;
    cv::rectangle(img2,
      cv::Point(camera2_points[i][0] - delta, camera2_points[i][1] - delta),
      cv::Point(camera2_points[i][0] + delta, camera2_points[i][1] + delta),
      cv::Scalar(0, 0, 255),
      5);
  }

  // Show images
  double image_width = img1.size().width;
  double image_height = img1.size().height;

  cv::Mat img1_resize;
  cv::resize(img1, img1_resize, cv::Size(image_width/4.0, image_height/4.0));
  cv::imshow("img1", img1_resize);
  // cv::waitKey();


  cv::Mat img2_resize;

  cv::Mat hom_img2(img2.clone());
  for (int i = 0; i < hom_points2.size(); ++i) {
    cv::drawMarker(hom_img2, hom_points2[i], cv::Scalar(0, 255.0, 0.0), cv::MARKER_CROSS, 20, 3);
  }
  cv::resize(hom_img2, img2_resize, cv::Size(image_width/4.0, image_height/4.0));
  cv::imshow("hom_img2", img2_resize);
  // cv::waitKey();

  hom_img2 = img2.clone();
  for (int i = 0; i < hom0_points2.size(); ++i) {
    cv::drawMarker(hom_img2, hom0_points2[i], cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_STAR, 20, 3);
  }
  cv::resize(hom_img2, img2_resize, cv::Size(image_width/4.0, image_height/4.0));
  cv::imshow("hom0_img2", img2_resize);
  

  cv::Mat warp1 = cv::Mat::zeros(img1.size(), img1.type());
  cv::warpPerspective(img1, warp1, hom_res, warp1.size());
  cv::Mat warp1_resize;
  cv::resize(warp1, warp1_resize, cv::Size(image_width/4.0, image_height/4.0));
  cv::imshow("Warp1", warp1_resize);
  // cv::waitKey();

  cv::warpPerspective(img1, warp1, hom_res0, warp1.size());
  cv::resize(warp1, warp1_resize, cv::Size(image_width/4.0, image_height/4.0));
  cv::imshow("Warp10", warp1_resize);
  cv::waitKey();
  


  

/*
  GLWindow gl_window("OpenGL: Two Cameras", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));
  std::shared_ptr<DObject> root = std::make_shared<DObject>();
  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(1.0, 50));
  std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(1.0f));
  root->AddChild(axes_obj);

  std::shared_ptr<ModelObject> debug_cube_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/debug_cube/debug_cube.obj"));
  debug_cube_obj->SetScale(glm::vec3(1.0f));
  debug_cube_obj->SetTranslation(glm::vec3(3.0f, 3.0f, 3.0f));
  root->AddChild(debug_cube_obj);





  // Camera Objects
  // float width_ratio = camera->GetImageWidth()/camera->GetImageHeight();
  CameraIntrinsics camera_intr = camera->GetCameraIntrinsics();

  std::shared_ptr<CameraObject> camera1_obj(new CameraObject(camera_intr));
  camera1_obj->SetTranslation(glm::vec3(3.0f, 0.7f, 1.0f));
  camera1_obj->SetRotation(static_cast<float>(-M_PI_2), 0.0f, static_cast<float>(-M_PI_2));
  // camera1_obj->SetImage(image1_path.string());
  camera1_obj->SetImage(img1);
  camera1_obj->SetImageTransparency(true);
  camera1_obj->SetImageAlpha(0.9);
  root->AddChild(camera1_obj);

  std::shared_ptr<CameraObject> camera2_obj(new CameraObject(camera_intr));
  camera2_obj->SetTranslation(glm::vec3(3.0f, -0.7f, 1.0f));
  camera2_obj->SetRotation(static_cast<float>(-M_PI_2), 0.0f, static_cast<float>(-M_PI_2));
  // camera2_obj->SetImage(image2_path.string());
  camera2_obj->SetImage(img2);
  camera2_obj->SetImageTransparency(true);
  camera2_obj->SetImageAlpha(0.9);
  root->AddChild(camera2_obj);


  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    // ====================== Render =====================
    renderer->Draw(floor_obj);

    renderer->Draw(root, true);

    // renderer->Draw(camera_obj, true);

    gl_window.RunLoop();
  }

  */

  return EXIT_SUCCESS;


}
