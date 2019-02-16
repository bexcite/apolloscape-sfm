#include <iostream>
#include <unordered_set>
#include <thread>

#include <boost/filesystem.hpp>

#include <cereal/cereal.hpp>
// #include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>
#include <glog/logging.h>

#define STRIP_FLAG_HELP 1    // this must go before the #include!
#include <gflags/gflags.h>

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


DEFINE_string(restore, "", "--restore=\"<filename>\" Saved SfM serialization"
                           " to continue SfM reconstruction pipeline");
DEFINE_string(output, "sfm_out.bin", "--output=\"<filename>\" : Destination"
                      " for SfM serialization");
DEFINE_bool(h, false, "Show help");

DECLARE_bool(help);
DECLARE_bool(helpshort);



namespace fs = boost::filesystem;

void MakeCameras(std::shared_ptr<DObject>& cameras,
                 const MapCameras& map_cameras,
                 const SfM3D& sfm,
                 const std::vector<std::shared_ptr<CameraObject> >& cameras_pool);


int main(int argc, char* argv[]) {
  std::cout << "Welcome 3D Reconstruction.\n";

  std::cout << "Concurrency = " << std::thread::hardware_concurrency() << std::endl;

  // Logger for Ceres Solver
  google::InitGoogleLogging(argv[0]);

  
  gflags::SetUsageMessage("SfM Reconstruction for Apolloscape ZPark dataset");
  gflags::SetVersionString("0.0.42");

  // Workaround for --help to show only --helpshort
  // https://github.com/gflags/gflags/issues/43#issuecomment-168280647
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (FLAGS_help || FLAGS_h) {
    FLAGS_help = false;
    FLAGS_helpshort = true;
  }
  gflags::HandleCommandLineHelpFlags();

  std::cout << "restore_flag=" << FLAGS_restore << std::endl;
  std::cout << "output_flag=" << FLAGS_output << std::endl;

  if (FLAGS_restore.empty()) {
    std::cout << "NO RESTORE\n";
  } else {
    std::cout << "TRY TO RESTORE FROM: " << FLAGS_restore << std::endl;
  }

  // gflags::ShutDownCommandLineFlags();
  // return EXIT_SUCCESS;
  

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

  if (FLAGS_restore.empty()) {
    // Create new run
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
      int p_camera_start = 21; //22 ==== 36 or 37 - 35
      int p_camera_finish = 25; //25 ===== 39 or 40  - 39

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

    // sfm.InitReconstruction();

  } else {

  // sfm.ReconstructAll();

  // {
  //   std::cout << "Serializing SFM!!!!\n";
  //   std::ofstream file("sfm_out.bin", std::ios::binary);
  //   cereal::BinaryOutputArchive archive(file);
  //   archive(sfm);
  //   sfm.PrintFinalStats();
  //   std::cout << "Serializing SFM!!!! - DONE\n";
  // }

  //SfM3D sfm;
  //{
    std::cout << "De-Serializing SFM!!!!\n";
    // std::ifstream file("sfm_out_2.bin", std::ios::binary);
    std::ifstream file(FLAGS_restore, std::ios::binary);
    cereal::BinaryInputArchive archive(file);
    archive(sfm);
    sfm.RestoreImages();
    sfm.PrintFinalStats();
    std::cout << "De-Serializing SFM!!!! - DONE\n";
  }


  // using namespace std::chrono;
  // auto t00 = high_resolution_clock::now();
  // sfm.ReconstructAll();
  // sfm.PrintFinalStats();
  // auto t11 = high_resolution_clock::now();
  // auto durt = duration_cast<microseconds>(t11 - t00);
  // std::cout << "TOTAL RECON_ALL TIME: " << durt.count() / 1e+6 << std::endl;
  // return EXIT_SUCCESS;

  // sfm.ReconstructAll();
  return EXIT_SUCCESS;


  std::thread recon_thread([&sfm]() {
    sfm.ReconstructAll();
    sfm.SetProcStatus(SfM3D::FINISH);
  });


  // std::cout << sfm << std::endl;
  // sfm.PrintFinalStats();



  // ====== Create Window and Renderer

  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(3.0f * kGlobalScale, 0.0f * kGlobalScale, 1.0f * kGlobalScale)); 
  camera->SetScale(kGlobalScale);

  GLWindow gl_window("OpenGL: 3D Reconstruction", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));
  std::shared_ptr<DObject> root = std::make_shared<DObject>();
  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(kGlobalScale, 60));
  // std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(kGlobalScale));
  // root->AddChild(axes_obj);

  // ========= 3D Recon Objects ===================
  std::shared_ptr<DObject> cameras(new DObject());
  std::shared_ptr<DObject> points_obj(nullptr);
  

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
    



  // Create all camera objects
  std::vector<std::shared_ptr<CameraObject> > cameras_pool(sfm.ImageCount());

  for (int i = 0; i < sfm.ImageCount(); ++i) {
    cv::Mat co_img = sfm.GetImage(i);
    // cv::resize(co_img, co_img, cv::Size(), 0.25, 0.25 /*, cv::INTER_AREA*/);
    CameraInfo cam_info = sfm.GetCameraInfo(i);
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
    // co->SetImage(img_points);
    co->SetImage(co_img);
    co->SetImageAlpha(0.99);
    cameras_pool[i] = co;
  }

  std::cout << "\ncameras_pool.size = " << cameras_pool.size() << std::endl;
  // std::cout << "cameras_pool[0] size = " << sizeof(cameras_pool[0]) << std::endl;


  // Get cameras with points
  // map: cam_id => vec of (keypoint_id, 3d_point_coords)
  // std::map<int, std::vector<std::pair<int, Point3DColor> > > map_cameras;
  // MapCameras map_cameras;


  std::vector<Point3DColor> glm_points;
  MapCameras map_cams;
  int lv = -1;
  sfm.GetMapPointsAndCameras(glm_points, map_cams, lv);
  MakeCameras(cameras, map_cams, sfm, cameras_pool);

  int origin_cam_id = map_cams.begin()->first;
  CameraInfo origin_cam_info = sfm.GetCameraInfo(origin_cam_id);
  

  typedef std::pair<const int, std::vector<std::pair<int, Point3DColor> > > MapCameraEl;

  // Set Origin to the Best Camera (with the most matches)
  // auto cam_max_points = std::max_element(map_cams.begin(),
  //     map_cams.end(),
  //     [](MapCameraEl& el1,
  //        MapCameraEl& el2) {
  //          return el1.second.size() < el2.second.size();
  //     });
  // int cam_max_points_id = cam_max_points->first;
  // std::cout << "cam_max_points_id = " << cam_max_points_id 
  //           << ", cam_size = " << cam_max_points->second.size() 
  //           << std::endl;
  // CameraInfo origin_cam_info = sfm.GetCameraInfo(cam_max_points_id);
  // camera->SetOrigin(origin_cam_info.translation); //glm::vec3(cam_info.translation)
  // camera->SetRotation(origin_cam_info.rotation_angles[0],
  //                     origin_cam_info.rotation_angles[1],
  //                     origin_cam_info.rotation_angles[2]);

  // std::vector<std::shared_ptr<CameraObject> > camera_refs;


  // cv windows are not working well with glfw windows
  cv::destroyAllWindows();

  // ========= View Debug Objects =================

  // std::shared_ptr<ModelObject> debug_cube_obj(
  //     ObjectFactory::CreateModelObject(
  //         "../data/objects/debug_cube/debug_cube.obj"));
  // debug_cube_obj->SetScale(glm::vec3(kGlobalScale));
  // debug_cube_obj->SetTranslation(glm::vec3(3.0f * kGlobalScale, 3.0f * kGlobalScale, 3.0f * kGlobalScale));
  // root->AddChild(debug_cube_obj);


  // == Change Camera Events ===
  // int current_camera_id = 0;
  // int current_camera_id = cam_max_points_id;
  int current_camera_id = 0;
  double last_camera_change = 0;
  auto change_camera = [&cameras,
                        &camera] (int camera_id) {
    std::cout << "camera_id = " << camera_id << std::endl;
    // CameraInfo cam_info = sfm.GetCameraInfo(camera_id);
    camera->SetOrigin(glm::vec3(cameras->GetChild(camera_id)->GetTranslation()));
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
      &last_camera_change, &cameras, &gl_window, // &used_views,
      &change_camera] (float dt) {
    int cameras_size = cameras->GetChildrenSize();
    const double key_rate = 0.2;
    double now = gl_window.GetTime();
    if (now - last_camera_change < key_rate) return;
    last_camera_change = now;
    // std::cout << "AddProcessInput!! == J = " << dt 
    //           << ", time = " << gl_window.GetTime() << std::endl;
    // do {
      current_camera_id = (current_camera_id + 1) % cameras_size;
    // } while (!used_views.count(current_camera_id));

    change_camera(current_camera_id);
  });

  gl_window.AddProcessInput(GLFW_KEY_K, [&current_camera_id,
      &last_camera_change, &cameras, &gl_window, // &used_views,
      &change_camera] (float dt) {
    int cameras_size = cameras->GetChildrenSize();
    const double key_rate = 0.2;
    double now = gl_window.GetTime();
    if (now - last_camera_change < key_rate) return;
    last_camera_change = now;
    // std::cout << "AddProcessInput!! == K = " << dt 
    //           << ", time = " << gl_window.GetTime() << std::endl;
    // do {
      current_camera_id = (current_camera_id + cameras_size - 1) % cameras_size;
    // } while (!used_views.count(current_camera_id));

    change_camera(current_camera_id);
  });
  // <<<<<< Change Camera Events


  // === Change Camera Alphas
  float cameras_alpha = 0.2f;
  auto change_alpha = [&cameras/*, &cameras_alpha*/](float cam_alpha) {
    auto set_alpha = [/*&cameras_alpha*/cam_alpha](std::shared_ptr<DObject> obj) {
      std::shared_ptr<CameraObject> co = std::static_pointer_cast<CameraObject>(obj);
      co->SetImageAlpha(/*cameras_alpha*/cam_alpha);
    };
    cameras->Apply(TAG_CAMERA_OBJECT, set_alpha);
  };

  // Apply it in order to make all cameras with the same alpha
  change_alpha(cameras_alpha);

  gl_window.AddProcessInput(GLFW_KEY_Z, [&cameras_alpha, &change_alpha](float dt) {
    cameras_alpha = std::max(cameras_alpha - 0.9f * dt, 0.0f);
    // std::cout << "AddProcessInput!! == Z = " << alpha << std::endl;
    change_alpha(cameras_alpha);
  });

  gl_window.AddProcessInput(GLFW_KEY_X, [&cameras_alpha, &change_alpha](float dt) {
    cameras_alpha = std::min(cameras_alpha + 0.9f * dt, 1.0f);
    // std::cout << "AddProcessInput!! == X = " << alpha << std::endl;
    change_alpha(cameras_alpha);
  });
  // <<<<<< Change Camera Alpha


  
  


  std::atomic<int> last_vis_version(0);

  
  // Fetch Points and Cameras for Drawing
  std::mutex cameras_points_mu;
  std::atomic<ThreadStatus> vis_prep_status(ThreadStatus::PROCESSING);

  std::thread vis_prep_thread([&vis_prep_status, &glm_points, &map_cams, &sfm,
                        &cameras_points_mu, &last_vis_version]() {
    
    while(vis_prep_status.load() != ThreadStatus::FINISH 
            && !sfm.IsFinished()) {
      
      std::vector<Point3DColor> glm_points_temp;
      MapCameras map_cams_temp;

      int lversion = last_vis_version.load();
      sfm.GetMapPointsAndCameras(glm_points_temp, map_cams_temp,
                                 lversion);
      last_vis_version.store(lversion);

      cameras_points_mu.lock();
      glm_points = glm_points_temp;
      map_cams = map_cams_temp;
      cameras_points_mu.unlock();

      std::cout << "\nFETCHED_VIS_VERSION = " << last_vis_version << std::endl;      
      // std::cout << "\n TICKING ... \n";
      //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  });


  
  long frames_cntr = 0;
  double dur_make_points = 0;
  double dur_make_cameras = 0;
  double dur_all = 0;

  int drawed_version = -1;

  // points_obj = std::shared_ptr<DObject>(
  //       ObjectFactory::CreatePoints(glm_points, true));
  // MakeCameras(cameras, map_cams, sfm, cameras_pool);
  //     change_alpha(cameras_alpha);

  double all_time = 0;

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;
    all_time += static_cast<double>(gl_window.delta_time);
    // std::cout << "==== START CYCLE: delta_time = " << all_time << std::endl;

    using namespace std::chrono;

    // ====================== Render =====================
    renderer->Draw(floor_obj);
    renderer->Draw(root, true);
    

    // Crazy drawing
    if (drawed_version < last_vis_version.load()) {
      // if(sfm.GetMapPointsVec(glm_points)) {
        // std::cout << "\n\nglm_points.size = " << glm_points.size() << std::endl;
      auto t00 = high_resolution_clock::now();
      cameras_points_mu.lock();
      auto t0 = high_resolution_clock::now();
      points_obj = std::shared_ptr<DObject>(
        ObjectFactory::CreatePoints(glm_points, true));
      auto t1 = high_resolution_clock::now();
      double dur_mp = duration_cast<microseconds>(t1-t0).count() / 1e+6;
      dur_make_points += dur_mp;
      // std::cout << "\nMAKE_POINTS_TIME = " << dur_mp << std::endl;
      
      // }
      
      // if(sfm.GetMapCamerasWithPointsVec(map_cams)) {
      auto t2 = high_resolution_clock::now();
      MakeCameras(cameras, map_cams, sfm, cameras_pool);
      change_alpha(cameras_alpha);
      auto t3 = high_resolution_clock::now();
      double dur = duration_cast<microseconds>(t3-t2).count() / 1e+6;
      dur_make_cameras += dur;
      // std::cout << "MAKE_CAMERAS_TIME = " << dur << std::endl;
      // }
      cameras_points_mu.unlock();
      auto t11 = high_resolution_clock::now();
      dur_all += duration_cast<microseconds>(t11-t00).count() / 1e+6;

      drawed_version = last_vis_version.load();
    }



    if (frames_cntr % 1000 == 0) {
      // std::cout << "\nMAKE_POINTS_TIME = " << dur_make_points / 1000.0 << std::endl;
      // std::cout << "MAKE_CAMERAS_TIME = " << dur_make_cameras / 1000.0 << std::endl;
      // std::cout << "FRAME_ALL_HARD_TIME = " << dur_all / 1000.0 << std::endl;
      dur_make_points = 0.0;
      dur_make_cameras = 0.0;
      dur_all = 0.0;
    }

    
    if (points_obj) {
      renderer->Draw(points_obj, false);

      // std::shared_ptr<DObject> points_obj1 = std::shared_ptr<DObject>(
      //   ObjectFactory::CreatePoints(glm_points, false));
      // renderer->Draw(points_obj1, false);
      

    }
    if (cameras) {
      renderer->Draw(cameras, true);
    }

    ++frames_cntr;
    gl_window.RunLoop();
    // std::cout << "<<< END CYCLE ===========" << std::endl;
  }

  // gl_window.Terminate();

  // first.detach();



  vis_prep_status.store(ThreadStatus::FINISH);
  sfm.SetProcStatus(SfM3D::FINISH);
  recon_thread.join();
  vis_prep_thread.join();


  std::string output_file = FLAGS_output;
  if (!FLAGS_restore.empty()) {
    output_file = FLAGS_restore;
  }


  {
    std::cout << "Serializing SFM!!!! to: " << output_file << std::endl;
    std::ofstream file(output_file, std::ios::binary);
    cereal::BinaryOutputArchive archive(file);
    archive(sfm);
    sfm.PrintFinalStats();
    std::cout << "Serializing SFM!!!! - DONE (" 
              << output_file << ")" << std::endl;
  }

  // Clear Gflags memory
  gflags::ShutDownCommandLineFlags();

  return EXIT_SUCCESS; 

}

void MakeCameras(std::shared_ptr<DObject>& cameras, 
                 const MapCameras& map_cameras,
                 const SfM3D& sfm,
                 const std::vector<std::shared_ptr<CameraObject> >& cameras_pool) {

  assert(cameras_pool.size() == sfm.ImageCount());

  using namespace std::chrono;
  auto t0 = high_resolution_clock::now();

  double dur_dkwr = 0.0;
  double dur_cc = 0.0;
  double dur_app = 0.0;
  double dur_gk = 0.0;
  double dur_gm = 0.0;

  cameras->ClearChildren();
  // === Draw Cameras ===
  for (auto& c: map_cameras) {
    
    // std::cout << "Camera: " << c.first << ": points = " << c.second.size() << std::endl;
    // PrintVec(" => ", c.second);

    const int& cam_id = c.first;  

    std::shared_ptr<CameraObject> co = cameras_pool[cam_id];

    /*
    auto t4_0 = high_resolution_clock::now();
    // cv::Mat co_img = sfm.GetImage(cam_id);
    auto t4_1 = high_resolution_clock::now();
    dur_gm += duration_cast<microseconds>(t4_1-t4_0).count() / 1e+6;

    auto t0_0 = high_resolution_clock::now();
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
    auto t0_1 = high_resolution_clock::now();
    dur_gk += duration_cast<microseconds>(t0_1-t0_0).count() / 1e+6;

    cv::Mat img_points;
    auto t1_0 = high_resolution_clock::now();
    // DrawKeypointsWithResize(co_img, kpoints, img_points, 0.3);
    auto t1_1 = high_resolution_clock::now();
    dur_dkwr += duration_cast<microseconds>(t1_1-t1_0).count() / 1e+6;
    

    CameraInfo cam_info = sfm.GetCameraInfo(cam_id);
    
    auto t2_0 = high_resolution_clock::now();
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
    // co->SetImage(img_points);
    co->SetImageAlpha(0.99);
    
    auto t2_1 = high_resolution_clock::now();
    dur_cc += duration_cast<microseconds>(t2_1-t2_0).count() / 1e+6;
    

    // co->AddProjectedPoints(glm_points);
    auto t3_0 = high_resolution_clock::now();
    co->AddProjectedPoints(points_3d);
    auto t3_1 = high_resolution_clock::now();
    dur_app += duration_cast<microseconds>(t3_1-t3_0).count() / 1e+6;
    */

    cameras->AddChild(co);
    
  }

  // std::cout << "\n   get_mat_image_TIME = " << dur_gm << std::endl;
  // std::cout << "\n   get_keypoints_TIME = " << dur_gk << std::endl;
  // std::cout << "\n   draw_keypoints_TIME = " << dur_dkwr << std::endl;
  // std::cout << "\n   camera_object_TIME = " << dur_cc << std::endl;
  // std::cout << "\n   add_projection_TIME = " << dur_app << std::endl;

  auto t1 = high_resolution_clock::now();
  auto dur_mc = duration_cast<microseconds>(t1 - t0).count() / 1e+6;
  // std::cout << "\nMAKE_CAMERAS_TIME = " << dur_mc << std::endl;

}