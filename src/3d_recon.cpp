#include <iostream>
#include <unordered_set>

#include <boost/filesystem.hpp>

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>
#include <glog/logging.h>

#include "cv_gl/camera.h"
#include "cv_gl/renderer.h"
#include "cv_gl/object_factory.hpp"
#include "cv_gl/gl_window.h"
#include "cv_gl/utils.hpp"
#include "cv_gl/sfm.h"
#include "cv_gl/serialization.hpp"
#include "cv_gl/ccomp.hpp"



const int kWindowWidth = 1226/2;
const int kWindowHeight = 1028/2;

const char kApolloDatasetPath[] = "../data/apolloscape";

const char kRoadId[] = "zpark-sample";
// const char kRecordId[] = "Record001";

const char kCamera1PoseFile[] = "Camera_1.txt";
const char kCamera2PoseFile[] = "Camera_2.txt";

const double kImageWidth = 2452.0;
const double kImageHeight = 2056.0;

// Used for grid visualization and camera speed
const float kGlobalScale = 100.0f;

const std::vector<std::string> kRecords = {
  "Record001",
  // "Record002",
  // "Record003",
  "Record004",
  // "Record006",
  // "Record007",
  // "Record008",
  // "Record009",
  // "Record010",
  // "Record011",
  // "Record012",
  // "Record013",
  // "Record014"
};

// An ID for Camera in a scene graph
#define TAG_CAMERA_OBJECT 10

namespace fs = boost::filesystem;


int main(int argc, char* argv[]) {
  std::cout << "Welcome 3D Reconstruction.\n";

  // Logger for Ceres Solver
  google::InitGoogleLogging(argv[0]);

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

  std::vector<CameraIntrinsics> camera_intrs = {intr1, intr2};

  SfM3D sfm(camera_intrs);

  for (auto record : kRecords) {
    std::cout << "==== r = " << record << std::endl;

    fs::path camera1_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
        / fs::path("pose") / fs::path(record) / fs::path(kCamera1PoseFile);
    fs::path camera2_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
        / fs::path("pose") / fs::path(record) / fs::path(kCamera2PoseFile);
    std::cout << "Camera 1 path: " << camera1_path << std::endl;
    std::cout << "Camera 2 path: " << camera2_path << std::endl;

    fs::path camera1_image_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
        / fs::path("image") / fs::path(record) / fs::path("Camera_1");
    fs::path camera2_image_path = fs::path(kApolloDatasetPath) / fs::path(kRoadId)
        / fs::path("image") / fs::path(record) / fs::path("Camera_2");

    std::vector<ImageData> camera1_poses = ReadCameraPoses(camera1_path,
                                                          camera1_image_path,
                                                          record, 1);
    std::vector<ImageData> camera2_poses = ReadCameraPoses(camera2_path,
                                                          camera2_image_path,
                                                          record, 2);

    

    std::cout << "sizes 1 = " << camera1_poses.size() << std::endl;
    std::cout << "sizes 2 = " << camera2_poses.size() << std::endl;

    std::cout << "Camera Poses 1: " << camera1_poses[1] << std::endl;
    std::cout << "Camera Poses 2: " << camera2_poses[1] << std::endl;

    // == Slice record - for testing ==
    int p_camera_pose = 0; // 24
    int p_camera_start = 22; //22 ==== 36 or 37 - 35
    int p_camera_finish = 27; //25 ===== 39 or 40  - 39

    p_camera_start = std::min(p_camera_start,
                              static_cast<int>(camera1_poses.size()));
    p_camera_finish = std::min(p_camera_finish,
                              static_cast<int>(camera1_poses.size()));

    std::vector<ImageData> camera1_poses_s, camera2_poses_s;
    camera1_poses_s.insert(camera1_poses_s.begin(),
                          camera1_poses.begin() + p_camera_start, 
                          camera1_poses.begin() + p_camera_finish);
    camera2_poses_s.insert(camera2_poses_s.begin(),
                          camera2_poses.begin() + p_camera_start, 
                          camera2_poses.begin() + p_camera_finish);
    sfm.AddImages(camera1_poses_s, camera2_poses_s, true, 1);

  }

  // return EXIT_SUCCESS;


  sfm.ExtractFeatures();

  sfm.MatchImageFeatures(60);

  sfm.InitReconstruction();



  sfm.ReconstructAll();


  std::cout << sfm << std::endl;
  sfm.PrintFinalStats();



  // ====== Create Window and Renderer

  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(3.0f * kGlobalScale, 0.0f * kGlobalScale, 1.0f * kGlobalScale)); 
  camera->SetScale(kGlobalScale);

  // camera->SetOrigin(glm::vec3(camera_origin_data.coords[3],
  //                             camera_origin_data.coords[4],
  //                             camera_origin_data.coords[5]));
  // camera->SetRotation(camera_origin_data.coords[0],
  //                     camera_origin_data.coords[1],
  //                     camera_origin_data.coords[2]);

  GLWindow gl_window("OpenGL: 3D Reconstruction", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));
  std::shared_ptr<DObject> root = std::make_shared<DObject>();
  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(kGlobalScale, 60));
  std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(kGlobalScale));
  root->AddChild(axes_obj);

  // ========= 3D Recon Objects ===================
  std::shared_ptr<DObject> cameras1(new DObject());
  
  

    // ======== Show Match =========
    // int win_x = 0;
    // int win_y = 20;
    // double win_scale = 0.25;
    // // == Show Camera Images (Keypoints) ==
    // cv::Mat img1 = images[ip.first].clone();
    // cv::Mat img2 = images[ip.second].clone();
    // cv::Mat img1_points, img2_points, img_matches;
    // DrawKeypointsWithResize(img1, features1.keypoints, img1_points, win_scale);
    // DrawKeypointsWithResize(img2, features2.keypoints, img2_points, win_scale);
    // std::vector<cv::KeyPoint> keypoints1, keypoints2;
    // for (size_t i = 0; i < matches.match.size(); ++i) {
    // // std::cout << i << " = " << good_matches[i].queryIdx << " : " << good_matches[i].trainIdx << std::endl;
    // // Add keypoints to output
    //   keypoints1.push_back(features1.keypoints[matches.match[i].queryIdx]);
    //   keypoints2.push_back(features2.keypoints[matches.match[i].trainIdx]);
    // }
    // DrawMatchesWithResize(img1, keypoints1, img2, keypoints2, img_matches, win_scale);
    // // Debug: Show points
    // ImShow("img1", img1_points);
    // cv::moveWindow("img1", win_x, win_y);
    // ImShow("img2", img2_points);
    // cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
    // // ImShow("mask", feature_mask, win_scale);
    // ImShow("img matches", img_matches);
    // cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
    // cv::waitKey();

    // Debug Draw
    // cv::drawMarker(img1_points, cv::Point2f(p1[0], p1[1]) * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
    // cv::drawMarker(img2_points, cv::Point2f(p2[0], p2[1]) * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
    // cv::drawMarker(img2_points, points2f[j] * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
    

  // Get cameras with points
  // map: cam_id => vec of (keypoint_id, 3d_point_coords)
  std::map<int, std::vector<std::pair<int, Point3DColor> > > map_cameras;
  sfm.GetMapCamerasWithPointsVec(map_cameras);

  typedef std::pair<const int, std::vector<std::pair<int, Point3DColor> > > MapCameraEl;

  // Set Origin to the Best Camera (with the most matches)
  auto cam_max_points = std::max_element(map_cameras.begin(),
      map_cameras.end(),
      [](MapCameraEl& el1,
         MapCameraEl& el2) {
           return el1.second.size() < el2.second.size();
      });
  int cam_max_points_id = cam_max_points->first;
  std::cout << "cam_max_points_id = " << cam_max_points_id 
            << ", cam_size = " << cam_max_points->second.size() 
            << std::endl;
  CameraInfo origin_cam_info = sfm.GetCameraInfo(cam_max_points_id);
  // camera->SetOrigin(origin_cam_info.translation); //glm::vec3(cam_info.translation)
  // camera->SetRotation(origin_cam_info.rotation_angles[0],
  //                     origin_cam_info.rotation_angles[1],
  //                     origin_cam_info.rotation_angles[2]);

  // std::vector<std::shared_ptr<CameraObject> > camera_refs;

  // === Draw Cameras ===
  for (auto& c: map_cameras) {
    
    std::cout << "Camera: " << c.first << ": points = " << c.second.size() << std::endl;
    // PrintVec(" => ", c.second);

    const int& cam_id = c.first;
    cv::Mat co_img = sfm.GetImage(cam_id);


    std::vector<cv::KeyPoint> kpoints;
    std::vector<Point3DColor> points_3d;
    for (size_t i = 0; i < c.second.size(); ++i) {

      cv::KeyPoint kp = sfm.GetKeypoint(cam_id, c.second[i].first);
      // std::cout << "kp.pt = " << kp.pt;
      // std::cout << ", size = " << kp.size;
      // std::cout << ", angle = " << kp.angle;
      // std::cout << std::endl;

      kpoints.push_back(kp);
      points_3d.push_back(c.second[i].second);
    }
    cv::Mat img_points;
    DrawKeypointsWithResize(co_img, kpoints, img_points, 0.3);

    CameraInfo cam_info = sfm.GetCameraInfo(cam_id);
    
    std::shared_ptr<CameraObject> co(
        new CameraObject(cam_info.intr, kImageWidth, kImageHeight));
    co->SetTag(TAG_CAMERA_OBJECT);
    co->SetImageTransparency(true);
    co->SetTranslation(cam_info.translation);
    co->SetRotation(cam_info.rotation_angles[0], 
                    cam_info.rotation_angles[1], 
                    cam_info.rotation_angles[2]);
    // co->SetImage(img2);
    // co->SetImage(img2_points);
    co->SetImage(img_points);
    co->SetImageAlpha(0.99);
    cameras1->AddChild(co);

    // co->AddProjectedPoints(glm_points);
    co->AddProjectedPoints(points_3d);
  }

  root->AddChild(cameras1);


  // cv windows are not working well with glfw windows
  cv::destroyAllWindows();

  // ========= View Debug Objects =================

  std::shared_ptr<ModelObject> debug_cube_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/debug_cube/debug_cube.obj"));
  debug_cube_obj->SetScale(glm::vec3(kGlobalScale));
  debug_cube_obj->SetTranslation(glm::vec3(3.0f * kGlobalScale, 3.0f * kGlobalScale, 3.0f * kGlobalScale));
  root->AddChild(debug_cube_obj);

  // == Change Camera Events ===
  // int current_camera_id = 0;
  int current_camera_id = cam_max_points_id;
  double last_camera_change = 0;  
  auto change_camera = [&sfm,
                        &camera] (int camera_id) {
    std::cout << "camera_id = " << camera_id << std::endl;
    CameraInfo cam_info = sfm.GetCameraInfo(camera_id);
    camera->SetOrigin(glm::vec3(cam_info.translation));
    // camera->SetRotation(cameras[camera_id].rotation_angles[0],
    //                     cameras[camera_id].rotation_angles[1],
    //                     cameras[camera_id].rotation_angles[2]);
  };

  // Set camera pose to the current_camera_id
  change_camera(current_camera_id);
  camera->SetRotation(origin_cam_info.rotation_angles[0],
                      origin_cam_info.rotation_angles[1],
                      origin_cam_info.rotation_angles[2]);

  int cameras_size = sfm.ImageCount();

  gl_window.AddProcessInput(GLFW_KEY_J, [&current_camera_id,
      &last_camera_change, &cameras_size, &gl_window, // &used_views,
      &change_camera] (float dt) {
    const double key_rate = 0.2;
    double now = gl_window.GetTime();
    if (now - last_camera_change < key_rate) return;
    last_camera_change = now;
    // std::cout << "AddProcessInput!! == J = " << dt 
    //           << ", time = " << gl_window.GetTime() << std::endl;
    // do {
      current_camera_id = (current_camera_id + 2) % cameras_size;
    // } while (!used_views.count(current_camera_id));

    change_camera(current_camera_id);
  });

  gl_window.AddProcessInput(GLFW_KEY_K, [&current_camera_id,
      &last_camera_change, &cameras_size, &gl_window, // &used_views,
      &change_camera] (float dt) {
    const double key_rate = 0.2;
    double now = gl_window.GetTime();
    if (now - last_camera_change < key_rate) return;
    last_camera_change = now;
    // std::cout << "AddProcessInput!! == K = " << dt 
    //           << ", time = " << gl_window.GetTime() << std::endl;
    // do {
      current_camera_id = (current_camera_id + cameras_size - 2) % cameras_size;
    // } while (!used_views.count(current_camera_id));

    change_camera(current_camera_id);
  });
  // <<<<<< Change Camera Events


  // === Change Camera Alphas
  float cameras_alpha = 0.9f;
  auto change_alpha = [&cameras1, &cameras_alpha]() {
    auto set_alpha = [&cameras_alpha](std::shared_ptr<DObject> obj) {
      std::shared_ptr<CameraObject> co = std::static_pointer_cast<CameraObject>(obj);
      co->SetImageAlpha(cameras_alpha);
    };
    cameras1->Apply(TAG_CAMERA_OBJECT, set_alpha);
  };
  change_alpha();

  gl_window.AddProcessInput(GLFW_KEY_Z, [&cameras_alpha, &change_alpha](float dt) {
    cameras_alpha = std::max(cameras_alpha - 0.9f * dt, 0.0f);
    // std::cout << "AddProcessInput!! == Z = " << alpha << std::endl;
    change_alpha();
  });

  gl_window.AddProcessInput(GLFW_KEY_X, [&cameras_alpha, &change_alpha](float dt) {
    cameras_alpha = std::min(cameras_alpha + 0.9f * dt, 1.0f);
    // std::cout << "AddProcessInput!! == X = " << alpha << std::endl;
    change_alpha();
  });
  // <<<<<< Change Camera Alpha


  std::vector<Point3DColor> glm_points;
  sfm.GetMapPointsVec(glm_points);

  // std::vector<glm::vec3> glm_only_points;
  // for (const auto& p : glm_points) {
  //   glm_only_points.push_back(p.pt);
  // }

  std::shared_ptr<DObject> points_obj(
      ObjectFactory::CreatePoints(glm_points, true/*, glm::vec4(1.0)*/));
  root->AddChild(points_obj);

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    // ====================== Render =====================
    renderer->Draw(floor_obj);

    renderer->Draw(root, true);

    // Crazy drawing  
    // renderer->Draw(points_obj, true);


    gl_window.RunLoop();
  }




  return EXIT_SUCCESS;

  


}