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
  double win_scale = 0.2;
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
    std::cout << "Fundamental Matrix: " << fund << std::endl;

    // == Extract Keypoints ==
    std::vector<cv::KeyPoint> kpoints1, kpoints2;
    GetMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2, fund);
    // GetMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2);
    std::cout << "kpoints1.size = " << kpoints1.size() << std::endl;
    std::cout << "kpoints2.size = " << kpoints2.size() << std::endl;


    
    // == Show Camera Images (Keypoints) ==
    cv::Mat img1_points, img2_points, img_matches;
    DrawKeypointsWithResize(img1, kpoints1, img1_points, win_scale);
    DrawKeypointsWithResize(img2, kpoints2, img2_points, win_scale);
    DrawMatchesWithResize(img1, kpoints1, img2, kpoints2, img_matches, win_scale);

    ImShow("img1", img1_points);
    cv::moveWindow("img1", win_x, win_y);
    ImShow("img2", img2_points);
    cv::moveWindow("img2", win_x + img1.size().width * 0.2, win_y);
    ImShow("img matches", img_matches);
    cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
    cv::waitKey();
    

    // == Triangulate Points =====
    std::vector<cv::Point2f> points1f;
    std::vector<cv::Point2f> points2f;
    KeyPointToPointVec(kpoints1, points1f);
    KeyPointToPointVec(kpoints2, points2f);

    cv::Mat points3d;
    TriangulatePoints(intr1, im_data1, points1f, intr2, im_data2, points2f, points3d);


    // === Visualize Cameras and Points
    std::shared_ptr<CameraObject> co1(new CameraObject(camera_intr));
    co1->SetTag(TAG_CAMERA_OBJECT);
    co1->SetImageTransparency(true);
    co1->SetTranslation(glm::vec3(im_data1.coords[3], im_data1.coords[4], im_data1.coords[5]));
    co1->SetRotation(im_data1.coords[0], im_data1.coords[1], im_data1.coords[2]);
    co1->SetImage(img1_points);
    co1->SetImageAlpha(0.7);
    cameras1->AddChild(co1);

    std::shared_ptr<CameraObject> co2(new CameraObject(camera_intr));
    co2->SetTag(TAG_CAMERA_OBJECT);
    co2->SetImageTransparency(true);
    co2->SetTranslation(glm::vec3(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5]));
    co2->SetRotation(im_data2.coords[0], im_data2.coords[1], im_data2.coords[2]);
    co2->SetImage(img2_points);
    co2->SetImageAlpha(0.7);
    cameras1->AddChild(co2);

    // Create points in glm::vec3
    std::cout << "points3d.rows = " << points3d.rows << std::endl;
    std::vector<glm::vec3> glm_points(points3d.rows);
    for (int i = 0; i < points3d.rows; ++i) {
      glm::vec3 v(
        points3d.at<float>(i, 0),
        points3d.at<float>(i, 1),
        points3d.at<float>(i, 2)
      );
      std::cout << "v = " << v << std::endl;
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