#include <iostream>

#include <boost/filesystem.hpp>

#include "cv_gl/camera.h"
#include "cv_gl/renderer.h"

#include "cv_gl/object_factory.hpp"

#include "cv_gl/gl_window.h"
#include "cv_gl/utils.hpp"
#include "cv_gl/sfm.h"

const int kWindowWidth = 1226/2;
const int kWindowHeight = 1028/2;

const char kApolloDatasetPath[] = "../data/apolloscape";

const char kRoadId[] = "zpark-sample";
const char kRecordId[] = "Record001";

const char kCamera1PoseFile[] = "Camera_1.txt";
const char kCamera2PoseFile[] = "Camera_2.txt";

const double kImageWidth = 2452.0;
const double kImageHeight = 2056.0;

const float kGlobalScale = 100.0f;

#define TAG_CAMERA_OBJECT 10

namespace fs = boost::filesystem;

int main(int argc, char* argv[]) {
  std::cout << "Welcome 3D Reconstruction.\n";

  CameraIntrinsics intr1;
  intr1.fx = 1450.317230113;
  intr1.fy = 1451.184836113;
  intr1.cx = 1244.386581025;
  intr1.cy = 1013.145997723;
  intr1.wr = kImageWidth/kImageHeight;

  CameraIntrinsics intr2;
  intr2.fx = 1448.572928508;
  intr2.fy = 1449.555907804;
  intr2.cx = 1250.940749515;
  intr2.cy = 1013.259732772;
  intr2.wr = kImageWidth/kImageHeight;


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

  std::cout << "sizes 1 = " << camera1_poses.size() << std::endl;
  std::cout << "sizes 2 = " << camera2_poses.size() << std::endl;

  std::cout << "Camera Poses 1: " << camera1_poses[1];
  std::cout << "Camera Poses 2: " << camera2_poses[1];

  int p_camera_pose = 24; // 24
  int p_camera_start = 22; //22
  int p_camera_finish = 25; //25

  const ImageData& camera_origin_data = camera1_poses[p_camera_pose];

  





  // ====== Create Window and Rendering

  // cv windows are not working well with glfw windows
  cv::destroyAllWindows();

  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(3.0f * kGlobalScale, 0.0f * kGlobalScale, 1.0f * kGlobalScale)); 
  camera->SetScale(kGlobalScale);

  camera->SetOrigin(glm::vec3(camera_origin_data.coords[3],
                              camera_origin_data.coords[4],
                              camera_origin_data.coords[5]));
  camera->SetRotation(camera_origin_data.coords[0],
                      camera_origin_data.coords[1],
                      camera_origin_data.coords[2]);

  GLWindow gl_window("OpenGL: 3D Reconstruction", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));
  std::shared_ptr<DObject> root = std::make_shared<DObject>();
  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(kGlobalScale, 50));
  std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(kGlobalScale));
  root->AddChild(axes_obj);


  // ========= 3D Recon Objects ===================
  // Camera Objects
  CameraIntrinsics camera_intr = camera->GetCameraIntrinsics();

  std::shared_ptr<DObject> cameras1(new DObject());
  std::shared_ptr<DObject> cameras2(new DObject());


  // ===== Extract features and Triangulate ================= 
  bool first_window = true;
  int win_x = 0;
  int win_y = 100;
  double win_scale = 0.3;
  int kpts_count = 0;
  for (size_t i = p_camera_start; i < camera1_poses.size(); ++i) {
    if (i == p_camera_finish) break;

    const ImageData& im_data1 = camera1_poses[i];
    const ImageData& im_data2 = camera2_poses[i];

    fs::path image1_path = camera1_image_path / fs::path(im_data1.filename); // 
    fs::path image2_path = camera2_image_path / fs::path(im_data2.filename); // camera2_poses[0].filename

    std::cout << "image1_path = " << image1_path << std::endl;
    std::cout << "image2_path = " << image2_path << std::endl;

    cv::Mat img1 = cv::imread(image1_path.string().c_str());
    cv::Mat img2 = cv::imread(image2_path.string().c_str());

    // == Show Camera Images ==
    // ImShow("img1", img1, win_scale);
    // cv::moveWindow("img1", win_x, win_y);
    // ImShow("img2", img2, win_scale);
    // cv::moveWindow("img2", win_x + img1.size().width * 0.2, win_y);
    // cv::waitKey();

    // == Compute Fundamental Matrix ==
    cv::Mat fund;
    fund = CalcFundamental(intr1, im_data1, intr2, im_data2);
    // std::cout << "Fundamental Matrix: " << fund << std::endl;

    


    // == Extract Keypoints ==
    std::vector<cv::KeyPoint> kpoints1, kpoints2;
    GetLineMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2, fund);
    // GetMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2 /*, fund*/);
    // GetMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2);
    std::cout << "kpoints1.size = " << kpoints1.size() << std::endl;
    std::cout << "kpoints2.size = " << kpoints2.size() << std::endl;
    kpts_count += kpoints1.size();

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

    kpoints1.erase(kpoints1.begin() + 1, kpoints1.end());
    cv::Matx31d kp1_1(kpoints1[0].pt.x, kpoints1[0].pt.y, 1.0);
    std::cout << "kp1_1 = " << kp1_1 << std::endl;

    cv::Mat kp1_1line = fund.t() * cv::Mat(kp1_1);

    std::cout << "kp1_1line = " << kp1_1line  << std::endl;
    std::vector<cv::Point2f> line_pts;

    if (abs(kp1_1line.at<double>(1)) > 10e-9) {
      cv::Point2f p1(0.0, - kp1_1line.at<double>(2) / kp1_1line.at<double>(1));
      if (p1.y >= 0.0 && p1.y <= kImageHeight && line_pts.size() < 2) {
        line_pts.push_back(p1);
      }
      cv::Point2f p2(kImageWidth, - (kp1_1line.at<double>(2) + kp1_1line.at<double>(0) * kImageWidth) / kp1_1line.at<double>(1));
      if (p2.y >= 0.0 && p2.y <= kImageHeight && line_pts.size() < 2) {
        line_pts.push_back(p2);
      }
    }
    if (abs(kp1_1line.at<double>(0)) > 10e-9 && line_pts.size() < 2) {
      cv::Point2f p3(- kp1_1line.at<double>(2) / kp1_1line.at<double>(0), 0.0);
      if (p3.x >= 0.0 && p3.x <= kImageWidth && line_pts.size() < 2) {
        line_pts.push_back(p3);
      }
      cv::Point2f p4(- (kp1_1line.at<double>(2) + kp1_1line.at<double>(1) * kImageHeight) / kp1_1line.at<double>(0), kImageHeight);
      if (p4.x >= 0.0 && p4.x <= kImageWidth && line_pts.size() < 2) {
        line_pts.push_back(p4);
      }
    }
    std::cout << "line_pts.size() = " << line_pts.size() << std::endl;    
    std::cout << "line_pts[0] = " << line_pts[0] << std::endl;
    std::cout << "line_pts[1] = " << line_pts[1] << std::endl;

    // == Distance to the kpoints2
    std::vector<double> dists;
    for (size_t j = 0; j < kpoints2.size(); ++j) {
      double d1 = abs(kp1_1line.at<double>(0) * kpoints2[j].pt.x + kp1_1line.at<double>(1) * kpoints2[j].pt.y + kp1_1line.at<double>(2));
      double d2 = sqrt(pow(kp1_1line.at<double>(0), 2) + pow(kp1_1line.at<double>(1), 2));
      double d = d1/d2;
      // std::cout << "d = " << d << std::endl;
      dists.push_back(d);
    }
    // std::sort(dists.begin(), dists.end());
    for (size_t j = 0; j < dists.size(); ++j) {
      if (dists[j] < 20) {
        std::cout << "dists[" << j << "] = " << dists[j] << std::endl;
        cv::drawMarker(img2, cv::Point2f(kpoints2[j].pt.x, kpoints2[j].pt.y), cv::Scalar(255.0, 100.0, 100.0), cv::MARKER_CROSS, 40, 8);
      }
    }

    // cv::Mat img1l;
    cv::drawMarker(img1, cv::Point2f(kpoints1[0].pt.x, kpoints1[0].pt.y), cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 20, 4);
    cv::line(img2, line_pts[0], line_pts[1], cv::Scalar(255.0, 0.0, 0.0), 2);
    */
  

    
    // == Show Camera Images (Keypoints) ==
    cv::Mat img1_points, img2_points, img_matches;
    DrawKeypointsWithResize(img1, kpoints1, img1_points, win_scale);
    DrawKeypointsWithResize(img2, kpoints2, img2_points, win_scale);
    DrawMatchesWithResize(img1, kpoints1, img2, kpoints2, img_matches, win_scale);
    
    

    
    
    // == Triangulate Points =====
    std::vector<cv::Point2f> points1f;
    std::vector<cv::Point2f> points2f;
    KeyPointToPointVec(kpoints1, points1f);
    KeyPointToPointVec(kpoints2, points2f);

    cv::Mat points3d;
    TriangulatePoints(intr1, im_data1, points1f, intr2, im_data2, points2f, points3d);
    
    
    // Backprojection error
    glm::dmat3 r1 = GetRotation(im_data1.coords[0], im_data1.coords[1], im_data1.coords[2]);
    glm::dmat3 r2 = GetRotation(im_data2.coords[0], im_data2.coords[1], im_data2.coords[2]);
    glm::dmat3 k1 = intr1.GetCameraMatrix();
    glm::dmat3 k2 = intr2.GetCameraMatrix();
    glm::dvec3 t1(im_data1.coords[3], im_data1.coords[4], im_data1.coords[5]);
    glm::dvec3 t2(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5]);
    double rep_err1 = 0.0;
    double rep_err2 = 0.0;
    for (size_t j = 0; j < points3d.rows; ++j) {
      glm::dvec3 x(
        points3d.at<float>(j, 0),
        points3d.at<float>(j, 1),
        points3d.at<float>(j, 2)
      );
      // std::cout << "xd = " << glm::to_string(x) << std::endl;
      glm::dvec3 p1 = k1 * glm::transpose(r1) * (x - t1);
      p1 = p1 / p1[2];
      glm::dvec3 p2 = k2 * glm::transpose(r2) * (x - t2);
      p2 = p2 / p2[2];
      // std::cout << "p1 = " << glm::to_string(p1) << std::endl;
      // std::cout << "p2 = " << glm::to_string(p2) << std::endl;
      glm::dvec2 dp1(points1f[j].x - p1[0], points1f[j].y - p1[1]);
      glm::dvec2 dp2(points2f[j].x - p2[0], points2f[j].y - p2[1]);
      // std::cout << "dp1 = " << glm::dot(dp1, dp1) << std::endl;
      // std::cout << "dp2 = " << glm::dot(dp2, dp2) << std::endl;
      rep_err1 += glm::dot(dp1, dp1);
      rep_err2 += glm::dot(dp2, dp2);

      // Debug Draw
      cv::drawMarker(img1_points, cv::Point2f(p1[0], p1[1]) * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
      cv::drawMarker(img2_points, cv::Point2f(p2[0], p2[1]) * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
      // cv::drawMarker(img2_points, points2f[j] * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
    }
    std::cout << "rep_err1 = " << rep_err1 << std::endl;
    std::cout << "rep_err2 = " << rep_err2 << std::endl;
    
    


    /*
    // Debug: Show points
    ImShow("img1", img1_points);
    cv::moveWindow("img1", win_x, win_y);
    ImShow("img2", img2_points);
    cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
    ImShow("img matches", img_matches);
    cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
    cv::waitKey();
    */
    

    // === Visualize Cameras and Points
    std::shared_ptr<CameraObject> co1(new CameraObject(camera_intr));
    co1->SetTag(TAG_CAMERA_OBJECT);
    co1->SetImageTransparency(true);
    co1->SetTranslation(glm::vec3(im_data1.coords[3], im_data1.coords[4], im_data1.coords[5]));
    co1->SetRotation(im_data1.coords[0], im_data1.coords[1], im_data1.coords[2]);
    // co1->SetImage(img1);
    co1->SetImage(img1_points);
    co1->SetImageAlpha(0.99);
    cameras1->AddChild(co1);

    std::shared_ptr<CameraObject> co2(new CameraObject(camera_intr));
    co2->SetTag(TAG_CAMERA_OBJECT);
    co2->SetImageTransparency(true);
    co2->SetTranslation(glm::vec3(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5]));
    co2->SetRotation(im_data2.coords[0], im_data2.coords[1], im_data2.coords[2]);
    // co2->SetImage(img2);
    co2->SetImage(img2_points);
    co2->SetImageAlpha(0.99);
    cameras1->AddChild(co2);

    
    // Create points in glm::vec3
    std::cout << "points3d.rows = " << points3d.rows << std::endl;
    std::vector<glm::vec3> glm_points(points3d.rows);
    for (int j = 0; j < points3d.rows; ++j) {
      glm::vec3 v(
        points3d.at<float>(j, 0),
        points3d.at<float>(j, 1),
        points3d.at<float>(j, 2)
      );
      // std::cout << "v = " << v << std::endl;
      glm_points.push_back(v);
    }
    std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(glm_points));
    // PrintVec("glm_points = ", glm_points);
    // std::cout << "glm_points = " << glm_points << std::endl;
    root->AddChild(points_obj);
    // std::cout << "points_obj = " << points_obj << std::endl;

    co1->AddProjectedPoints(glm_points);
    co2->AddProjectedPoints(glm_points);
    

  }

  root->AddChild(cameras1);
  root->AddChild(cameras2);

  std::cout << "kpts_count = " << kpts_count << std::endl;


  // cv windows are not working well with glfw windows
  cv::destroyAllWindows();

  // ========= View Debug Objects =================

  std::shared_ptr<ModelObject> debug_cube_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/debug_cube/debug_cube.obj"));
  debug_cube_obj->SetScale(glm::vec3(kGlobalScale));
  debug_cube_obj->SetTranslation(glm::vec3(3.0f * kGlobalScale, 3.0f * kGlobalScale, 3.0f * kGlobalScale));
  root->AddChild(debug_cube_obj);

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    // ====================== Render =====================
    renderer->Draw(floor_obj);

    renderer->Draw(root, true);


    gl_window.RunLoop();
  }


  return EXIT_SUCCESS;


}