#include <iostream>
#include <unordered_set>

#include <boost/filesystem.hpp>

#include "cv_gl/camera.h"
#include "cv_gl/renderer.h"

#include "cv_gl/object_factory.hpp"

#include "cv_gl/gl_window.h"
#include "cv_gl/utils.hpp"
#include "cv_gl/sfm.h"

#include "cv_gl/ccomp.hpp"

#include <glog/logging.h>

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

  // For Ceres Solver
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

  int p_camera_pose = 37; // 24
  int p_camera_start = 20; //22 ==== 36 or 37
  int p_camera_finish = 27; //25 ===== 39 or 40

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

  
  

  // Expect only paired images
  assert(camera1_poses.size() == camera2_poses.size());

  // === Extract features from images ===
  std::vector<Features> image_features;
  std::vector<CameraInfo> cameras;
  std::vector<cv::Mat> images;

  std::cout << "Extract Features: \n";
  for (size_t i = p_camera_start; i < camera1_poses.size(); ++i) {
    if (p_camera_finish == i) break;
    std::cout << "Camera 1: image " << i << std::endl;
    const ImageData& im_data = camera1_poses[i];
    cv::Mat img;
    Features features;
    CameraInfo camera_info;
    ExtractFeaturesAndCameraInfo(camera1_image_path.string(), im_data, intr1, img, features, camera_info);
    images.emplace_back(img);
    image_features.emplace_back(features);
    cameras.emplace_back(camera_info);
  }
  for (size_t i = p_camera_start; i < camera2_poses.size(); ++i) {
    if (p_camera_finish == i) break;
    std::cout << "Camera 2: image " << i << std::endl;
    const ImageData& im_data = camera2_poses[i];
    cv::Mat img;
    Features features;
    CameraInfo camera_info;
    ExtractFeaturesAndCameraInfo(camera2_image_path.string(), im_data, intr2, img, features, camera_info);
    images.emplace_back(img);
    image_features.emplace_back(features);
    cameras.emplace_back(camera_info);
  }

  std::cout << "image_features.size = " << image_features.size() << std::endl;
  std::cout << "cameras.size = " << cameras.size() << std::endl;
  std::cout << "images.size = " << images.size() << std::endl;


  // === Image Pairs to match ===
  std::vector<ImagePair> pairs;
  int look_back = 5;
  int cam_size = images.size() / 2;
  for (int i = 0; i < images.size() / 2; ++i ) {
    pairs.push_back({ i, i + cam_size });
    for (int j = i - 1; j >= std::max(0, i - look_back); --j) {
      pairs.push_back({ i, j });
      pairs.push_back({ i, j + cam_size });
    }
  }
  // == All Pairs
  // for (int i = 0; i < images.size()-1; ++i) {
  //   for (int j = i + 1; j < images.size(); ++j) {
  //     pairs.push_back({ i, j });
  //   }
  // }
  for (const ImagePair& ip : pairs) {
    std::cout << "pair: " << ip.first << " - " << ip.second << std::endl;
  }

  
  CComponents<std::pair<int, int> > ccomp;

  // == Match Features Between Image Pairs
  std::vector<Matches> image_matches;
  std::map<std::pair<int, int>, int> matches_index;
  int total_matched_points = 0;
  int cl = 0;
  int longest = 0;
  int most_match = 0;
  int most_match_id = -1;
  for (const ImagePair& ip : pairs) {
    Features& features1 = image_features[ip.first];
    Features& features2 = image_features[ip.second];
    CameraInfo& camera_info1 = cameras[ip.first];
    CameraInfo& camera_info2 = cameras[ip.second];

    Matches matches;
    matches.image_index.first = ip.first;
    matches.image_index.second = ip.second;
    ComputeLineKeyPointsMatch(features1, camera_info1, features2, camera_info2, matches);

    std::cout << "Match: " << ip.first << " - " << ip.second 
              << " : matches.size = " << matches.match.size() << std::endl;

    // Don't add empty or small matches
    if (matches.match.size() < 10) { 
      continue;
    }

    image_matches.emplace_back(matches);

    // put index
    auto p1 = std::make_pair(ip.first, ip.second);
    auto p2 = std::make_pair(ip.second, ip.first);
    matches_index.insert(
        std::make_pair(p1, image_matches.size() - 1));
    matches_index.insert(
        std::make_pair(p2, image_matches.size() - 1));

    if (matches.match.size() > most_match) {
      most_match = matches.match.size();
      most_match_id = image_matches.size() - 1;
    }

    total_matched_points += matches.match.size();
    std::cout << "total_matched_points = " << total_matched_points << std::endl;

    for (int i = 0; i < matches.match.size(); ++i) {
      std::pair<int, int> p1 = std::make_pair(
          matches.image_index.first,
          matches.match[i].queryIdx);
      std::pair<int, int> p2 = std::make_pair(
          matches.image_index.second, 
          matches.match[i].trainIdx);
      // std::cout << "union: " << p1 << ", " << p2 << std::endl;
      ccomp.Union(p1, p2);
    }

    // TODO: Extract from the loop
    /*
    std::cout << "ccomp.count = " << ccomp.Count() << std::endl;
    std::vector<int> comp_ids = ccomp.GetComponentIds();
    cl = 0;
    longest = 0;
    for (auto c : comp_ids) {
      auto elems = ccomp.GetElementsById(c);
      if (elems.size() > longest) {
        longest = elems.size();
        cl = c;
      }
    }
    std::cout << "longest = " << longest << ", id = " << cl << std::endl;
    */

    /*
    // ======== Show Match =========
    int win_x = 0;
    int win_y = 20;
    double win_scale = 0.25;
    // == Show Camera Images (Keypoints) ==
    cv::Mat img1 = images[ip.first].clone();
    cv::Mat img2 = images[ip.second].clone();
    cv::Mat img1_points, img2_points, img_matches;
    DrawKeypointsWithResize(img1, features1.keypoints, img1_points, win_scale);
    DrawKeypointsWithResize(img2, features2.keypoints, img2_points, win_scale);
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    for (size_t i = 0; i < matches.match.size(); ++i) {
    // std::cout << i << " = " << good_matches[i].queryIdx << " : " << good_matches[i].trainIdx << std::endl;
    // Add keypoints to output
      keypoints1.push_back(features1.keypoints[matches.match[i].queryIdx]);
      keypoints2.push_back(features2.keypoints[matches.match[i].trainIdx]);
    }
    DrawMatchesWithResize(img1, keypoints1, img2, keypoints2, img_matches, win_scale);
    // Debug: Show points
    ImShow("img1", img1_points);
    cv::moveWindow("img1", win_x, win_y);
    ImShow("img2", img2_points);
    cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
    // ImShow("mask", feature_mask, win_scale);
    ImShow("img matches", img_matches);
    cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
    cv::waitKey();
    */

  }


  // most_match_id = 2; // 2

  std::cout << "total_matched_points = " << total_matched_points << std::endl;
  std::cout << "most_match = " << most_match 
            << ", " << image_matches[most_match_id].image_index.first 
            << " - " << image_matches[most_match_id].image_index.second
            << std::endl;


  // std::unordered_set<int> processed_views;
  std::unordered_set<int> used_views;


  std::unordered_set<int> todo_views;
  for (size_t i = 0; i < images.size(); ++i) {
    todo_views.insert(i);
  }
  
  // == Show Image Pair with best matches ==
  int img1_id = image_matches[most_match_id].image_index.first;
  int img2_id = image_matches[most_match_id].image_index.second;
  cv::Mat img1 = images[img1_id].clone();
  cv::Mat img2 = images[img2_id].clone();
  double win_scale = 0.25;
  int win_x = 0;
  int win_y = 10;
  // ImShowMatchesWithResize(img1, image_features[img1_id].keypoints,
  //                         img2, image_features[img2_id].keypoints,
  //                         image_matches[most_match_id].match,
  //                         // EmptyMatch(),
  //                         win_scale, win_x, win_y);
  // cv::waitKey();


  // == Camera images pics ==
  std::vector<cv::KeyPoint> kpoints1, kpoints2;
  for (size_t i = 0; i < image_matches[most_match_id].match.size(); ++i) {
    kpoints1.push_back(image_features[img1_id].keypoints[image_matches[most_match_id].match[i].queryIdx]);
    kpoints2.push_back(image_features[img2_id].keypoints[image_matches[most_match_id].match[i].trainIdx]);
  }
  cv::Mat img1_points, img2_points;
  DrawKeypointsWithResize(img1, kpoints1, img1_points, win_scale);
  DrawKeypointsWithResize(img2, kpoints2, img2_points, win_scale);
  
  /*
  std::vector<cv::KeyPoint> kpoints1, kpoints2;
  for (size_t i = 0; i < image_matches[most_match_id].match.size(); ++i) {
    kpoints1.push_back(image_features[img1_id].keypoints[image_matches[most_match_id].match[i].queryIdx]);
    kpoints2.push_back(image_features[img2_id].keypoints[image_matches[most_match_id].match[i].trainIdx]);
  }

  cv::Mat img1_points, img2_points, img_matches;
  DrawKeypointsWithResize(img1, kpoints1, img1_points, win_scale);
  DrawKeypointsWithResize(img2, kpoints2, img2_points, win_scale);
  DrawMatchesWithResize(img1, image_features[img1_id].keypoints, 
                        img2, image_features[img2_id].keypoints, 
                        img_matches, 
                        win_scale, image_matches[most_match_id].match);
  ImShow("img1", img1_points);
  cv::moveWindow("img1", win_x, win_y);
  ImShow("img2", img2_points);
  cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
  // ImShow("img_matches", img_matches);
  // cv::moveWindow("img_matches", win_x, win_y + img1_points.size().height + 20);
  */
  // cv::waitKey();
  


  // == Triangulate Points =====
  std::vector<cv::Point2f> points1f, points2f;
  KeyPointsToPointVec(image_features[img1_id].keypoints, image_features[img2_id].keypoints,
                      image_matches[most_match_id].match, points1f, points2f);

  cv::Mat points3d;
  TriangulatePoints(cameras[img1_id], points1f, cameras[img2_id], points2f, points3d);


  std::cout << "points3d.size = " << points3d.rows << ", " << points3d.cols << std::endl << std::flush;
  std::cout << "points3d.c = " << points3d.channels() << std::endl << std::flush;
  std::cout << "points3d.type = " << points3d.type() << std::endl << std::flush;
  std::cout << "points3d.row(1) = " << points3d.row(1) << std::endl << std::flush;

  // Backprojection error
  cv::Mat proj1 = GetProjMatrix(cameras[img1_id]);
  cv::Mat proj2 = GetProjMatrix(cameras[img2_id]);

  std::vector<double> errs1 = GetReprojectionErrors(points1f, proj1, points3d);
  std::vector<double> errs2 = GetReprojectionErrors(points2f, proj2, points3d);

  // Z distance of points from camera
  std::vector<double> cam1_zdist = GetZDistanceFromCamera(cameras[img1_id],
                                                          points3d);
  std::vector<double> cam2_zdist = GetZDistanceFromCamera(cameras[img2_id],
                                                          points3d);

  // for (size_t i = 0; i < cam1_zdist.size(); ++i) {
  //   std::cout << "p = " << i << " : " << cam1_zdist[i] << std::endl;
  // }
  // << [end front back]

  std::cout << "AFTER points3d.row(0) = " << points3d.row(0) << std::endl << std::flush;
  std::cout << "AFTER points3d.size = " << points3d.rows << ", " << points3d.cols << std::endl << std::flush;

  double rep_err1 = 0.0;
  double rep_err2 = 0.0;

  Map3D map;

  // std::vector<cv::Point3d> points3d_good;
  for (size_t i = 0; i < errs1.size(); ++i) {
    rep_err1 += errs1[i];
    rep_err2 += errs2[i];
    // if (errs1[i] < 10.0 && errs2[i] < 10.0) {
      // points3d_good.push_back(
      //   cv::Point3d(
      //     points3d.at<float>(i, 0),
      //     points3d.at<float>(i, 1),
      //     points3d.at<float>(i, 2)
      //   ));

    if (errs1[i] > 3.0 || errs2[i] > 3.0           // reprojection error too big
        || cam1_zdist[i] < 0 || cam2_zdist[i] < 0  // points on the back of the camera
        || points3d.at<float>(i, 2) < 38.0) {      // it's out of the land
      // std::cout << i << " [SKIP] : errs1, errs2 = " << errs1[i]
      //           << ", " << errs2[i] << std::endl;
      continue;
    }

      // Add point to the map
    WorldPoint3D wp;
    wp.pt = cv::Point3d(
        points3d.at<float>(i, 0),
        points3d.at<float>(i, 1),
        points3d.at<float>(i, 2));
    wp.views[image_matches[most_match_id].image_index.first] 
        = image_matches[most_match_id].match[i].queryIdx;
    wp.views[image_matches[most_match_id].image_index.second] 
        = image_matches[most_match_id].match[i].trainIdx;
    map.push_back(wp);
      
    // }
  }

  // for (auto& wp : map) {
  //   std::cout << "mp: " << wp << std::endl;
  // }


  std::cout << "rep_err1 = " << rep_err1 << std::endl;
  std::cout << "rep_err2 = " << rep_err2 << std::endl;
  std::cout << "map.size = " << map.size() << std::endl;

  double all_error;
  all_error = GetReprojectionError(map, cameras, image_features);
  std::cout << "all_error = " << all_error << std::endl;

  // == Optimize Bundle ==
  // TODO!!!!!!!!!!!!!
  OptimizeBundle(map, cameras, image_features);

  all_error = GetReprojectionError(map, cameras, image_features);
  std::cout << "all_error = " << all_error << std::endl;

  // processed_views.insert(img1_id);
  // processed_views.insert(img2_id);
  used_views.insert(img1_id);
  used_views.insert(img2_id);

  todo_views.erase(img1_id);
  todo_views.erase(img2_id);

  
  
  

  while (todo_views.size() > 0) {

    // TODO: Get next best view
    int next_img_id = GetNextBestView(map, todo_views, 
        ccomp, image_matches, matches_index);

    if (next_img_id < 0) {
      next_img_id = (* todo_views.begin());
      todo_views.erase(next_img_id);
      continue;
    }
    

    if (next_img_id >= 0) {
      std::cout << "====> Process img_id = " << next_img_id << " (";
      std::cout << "todo_views.size = " << todo_views.size();
      std::cout <<  ") ... \n"; 

      Map3D view_map;

      // Pairwise use next_img_id and used_views
      for (auto view_it = used_views.begin(); view_it != used_views.end(); ++view_it) {
        int view_id = (* view_it);
        // std::cout << "==> Triangulate pair = " << next_img_id << ", "
        //           << view_id << std::endl;

        int first_id = next_img_id;
        int second_id = view_id;

        // std::cout << "first, second = " << first_id << ", "
        //           << second_id << std::endl;

        auto m = matches_index.find(std::make_pair(first_id, second_id));
        if (m == matches_index.end()) {
          // std::cout << "skip pair ...\n";
          continue;
        }

        int matches_id = m->second;
        Matches& matches = image_matches[matches_id];
        // std::cout << "matches_id = " << matches_id
        //           << ", matches.size = " << matches.match.size() << std::endl;
        
        if (matches.image_index.first != first_id) {
          std::swap(first_id, second_id);
          // std::cout << "SWAP: first, second = " << first_id << ", "
          //         << second_id << std::endl;
          // std::cout << "IMAGE_INDEX: first, second = " << matches.image_index.first << ", "
          //         << matches.image_index.second << std::endl;
        }

        // == Triangulate Points =====
        std::vector<cv::Point2f> points1f, points2f;
        KeyPointsToPointVec(image_features[first_id].keypoints,
                            image_features[second_id].keypoints,
                            matches.match, points1f, points2f);

        cv::Mat points3d;
        TriangulatePoints(cameras[first_id], points1f, 
                          cameras[second_id], points2f, 
                          points3d);


        // std::cout << "points3d.size = " << points3d.rows << ", " 
        //           << points3d.cols << std::endl << std::flush;
        // std::cout << "points3d.c = " << points3d.channels() 
        //           << std::endl << std::flush;
        // std::cout << "points3d.type = " << points3d.type() 
        //           << std::endl << std::flush;
        // std::cout << "points3d.row(1) = " << points3d.row(1) 
        //           << std::endl << std::flush;

        // Backprojection error
        cv::Mat proj1 = GetProjMatrix(cameras[first_id]);
        cv::Mat proj2 = GetProjMatrix(cameras[second_id]);

        std::vector<double> errs1 = GetReprojectionErrors(points1f, 
                                                          proj1, 
                                                          points3d);
        std::vector<double> errs2 = GetReprojectionErrors(points2f, 
                                                          proj2, 
                                                          points3d);

        // Z distance of points from camera
        std::vector<double> cam1_zdist = GetZDistanceFromCamera(cameras[first_id],
                                                                points3d);
        std::vector<double> cam2_zdist = GetZDistanceFromCamera(cameras[second_id],
                                                                points3d);

        double rep_err1 = 0.0;
        double rep_err2 = 0.0;
        for (size_t i = 0; i < errs1.size(); ++i) {
          // std::cout << i << ": errs1, errs2 = " << errs1[i]
          //           << ", " << errs2[i] << std::endl;
          rep_err1 += errs1[i];
          rep_err2 += errs2[i];

          if (errs1[i] > 3.0 || errs2[i] > 3.0           // reprojection error too big
              || cam1_zdist[i] < 0 || cam2_zdist[i] < 0  // points on the back of the camera
              || points3d.at<float>(i, 2) < 38.0) {      // it's out of the land
            // std::cout << i << " [SKIP] : errs1, errs2 = " << errs1[i]
            //           << ", " << errs2[i] << std::endl;
            continue;
          }


            // Add point to the map
          WorldPoint3D wp;
          wp.pt = cv::Point3d(
              points3d.at<float>(i, 0),
              points3d.at<float>(i, 1),
              points3d.at<float>(i, 2));
          wp.views[first_id]
              = matches.match[i].queryIdx;
          wp.views[second_id] 
              = matches.match[i].trainIdx;
          view_map.push_back(wp);
            
          // }
        }

        // std::cout << "rep_err1 = " << rep_err1 << std::endl;
        // std::cout << "rep_err2 = " << rep_err2 << std::endl;
        std::cout << "view_map.size = " << view_map.size() << std::endl;

        // for (auto& wp : view_map) {
        //   std::cout << "view_mp: " << wp << std::endl;
        // }


      }

      double all_error;

      
      all_error = GetReprojectionError(view_map, cameras, image_features);
      // std::cout << "view_error = " << all_error << std::endl;
      // == Optimize Bundle ==
      OptimizeBundle(view_map, cameras, image_features);
      all_error = GetReprojectionError(view_map, cameras, image_features);
      // std::cout << "view_error = " << all_error << std::endl;      


      MergeToTheMap(map, view_map, ccomp);


      all_error = GetReprojectionError(map, cameras, image_features);
      std::cout << "map_error = " << all_error << std::endl;
      // == Optimize Bundle ==
      OptimizeBundle(map, cameras, image_features);
      all_error = GetReprojectionError(map, cameras, image_features);
      std::cout << "map_error = " << all_error << std::endl;


      used_views.insert(next_img_id);
      todo_views.erase(next_img_id);
    }
  }

  
  

  // OptimizeBundle(map, cameras, image_features);



 
 

  // Debug Draw
  // cv::drawMarker(img1_points, cv::Point2f(p1[0], p1[1]) * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
  // cv::drawMarker(img2_points, cv::Point2f(p2[0], p2[1]) * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);
  // cv::drawMarker(img2_points, points2f[j] * win_scale, cv::Scalar(255.0, 0.0, 0.0), cv::MARKER_CROSS, 5, 1);


  /*
  // === Show All Tracks From Longest ==
  std::vector<int> comp_ids = ccomp.GetComponentIds();
  std::vector<std::pair<int, int> > comp_counts;
  for (auto c : comp_ids) {
    auto elems = ccomp.GetComponentsById(c);
    comp_counts.push_back({c, elems.size()});
  }
  std::sort(comp_counts.begin(), comp_counts.end(), 
      [](std::pair<int, int>& a, std::pair<int, int>& b) {
          return a.second < b.second; 
      });
  std::cout << "top: " << comp_counts[0].first << ", " << comp_counts[0].second << std::endl;

  for (auto it = comp_counts.rbegin(); it != comp_counts.rend(); ++it) {
    auto elems = ccomp.GetComponentsById((*it).first);
    std::sort(elems.begin(), elems.end());
    std::cout << "elems (" << (*it).first << " - " << elems.size() << ") = ";
    double win_scale = 0.3;
    for (auto el : elems) {
      std::cout << el << ", " << std::flush;
      cv::Mat img = images[el.first].clone();
      std::vector<cv::KeyPoint> kpts = {image_features[el.first].keypoints[el.second]};
      cv::Mat img_points;
      DrawKeypointsWithResize(img, kpts, img_points, win_scale);
      ImShow("img", img_points);
      cv::waitKey();
    }
    std::cout << std::endl;
  }
  */




  //std::cout << "longest = " << longest << ", id = " << cl << std::endl;
  // auto elems = ccomp.GetComponentsById(cl);
  // std::sort(elems.begin(), elems.end());
  // std::cout << "elems (" << cl << ") = ";
  // double win_scale = 0.3;
  // for (auto el : elems) {
  //   std::cout << el << ", ";
  //   cv::Mat img = images[el.first].clone();
  //   std::vector<cv::KeyPoint> kpts = {image_features[el.first].keypoints[el.second]};
  //   cv::Mat img_points;
  //   DrawKeypointsWithResize(img, kpts, img_points, win_scale);
  //   ImShow("img", img_points);
  //   cv::waitKey();
  // }


  /*
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
    // GetMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2 ); // fund
    // GetMatchedSURFKeypoints(img1, kpoints1, img2, kpoints2);
    std::cout << "kpoints1.size = " << kpoints1.size() << std::endl;
    std::cout << "kpoints2.size = " << kpoints2.size() << std::endl;
    kpts_count += kpoints1.size();

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
    

    
    // Debug: Show points
    // ImShow("img1", img1_points);
    // cv::moveWindow("img1", win_x, win_y);
    // ImShow("img2", img2_points);
    // cv::moveWindow("img2", win_x + img1_points.size().width, win_y);
    // ImShow("img matches", img_matches);
    // cv::moveWindow("img matches", win_x, win_y + img1_points.size().height + 20);
    // cv::waitKey();
    
    */

    /*
    // === Visualize Cameras and Points
    // std::shared_ptr<CameraObject> co1(new CameraObject(camera_intr));
    std::shared_ptr<CameraObject> co1(
        new CameraObject(intr1, kImageWidth, kImageHeight));
    co1->SetTag(TAG_CAMERA_OBJECT);
    co1->SetImageTransparency(true);
    // co1->SetTranslation(glm::vec3(im_data1.coords[3], im_data1.coords[4], im_data1.coords[5]));
    co1->SetTranslation(cameras[img1_id].translation);
    // co1->SetRotation(im_data1.coords[0], im_data1.coords[1], im_data1.coords[2]);
    co1->SetRotation(cameras[img1_id].rotation_angles[0], cameras[img1_id].rotation_angles[1], cameras[img1_id].rotation_angles[2]);
    // co1->SetImage(img1);
    co1->SetImage(img1_points);
    co1->SetImageAlpha(0.99);
    cameras1->AddChild(co1);

    // std::cout << "co1 = " << co1 << std::endl;

    std::shared_ptr<CameraObject> co2(
        new CameraObject(intr2, kImageWidth, kImageHeight));
    co2->SetTag(TAG_CAMERA_OBJECT);
    co2->SetImageTransparency(true);
    // co2->SetTranslation(glm::vec3(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5]));
    co2->SetTranslation(cameras[img2_id].translation);
    // co2->SetRotation(im_data2.coords[0], im_data2.coords[1], im_data2.coords[2]);
    co2->SetRotation(cameras[img2_id].rotation_angles[0], cameras[img2_id].rotation_angles[1], cameras[img2_id].rotation_angles[2]);
    // co2->SetImage(img2);
    co2->SetImage(img2_points);
    co2->SetImageAlpha(0.99);
    cameras1->AddChild(co2);
    */

    // std::cout << "co2 = " << co2 << std::endl;


    /*()  
    // Create points in glm::vec3
    // std::cout << "points3d.rows = " << points3d.rows << std::endl;
    // std::vector<glm::vec3> glm_points(points3d.rows);
    std::vector<glm::vec3> glm_points(points3d_good.size());
    for (int j = 0; j < points3d_good.size(); ++j) {
      glm::vec3 v(points3d_good[j].x, points3d_good[j].y, points3d_good[j].z);
      // std::cout << "v = " << v << std::endl;
      glm_points.push_back(v);
    }
    std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(glm_points));
    // PrintVec("glm_points = ", glm_points);
    // std::cout << "glm_points = " << glm_points << std::endl;
    root->AddChild(points_obj);
    // std::cout << "points_obj = " << points_obj << std::endl;
    */

    // co1->AddProjectedPoints(glm_points);
    // co2->AddProjectedPoints(glm_points);    

  // std::cout << "Map Points: \n";
  // for (size_t i = 0; i < map.size(); ++i) {
  //   std::cout << i << ": " << map[i] << std::endl;
  // }


  all_error = GetReprojectionError(map, cameras, image_features);
  std::cout << "FINAL_error = " << all_error << std::endl;
  std::cout << "USED_views = " << used_views.size()
            << " out of " << images.size() << std::endl;
  std::cout << "FINAL_map.size = " << map.size() << " points" << std::endl;


  // map cam_id => vec of (keypoint_id, 3d_point_coords)
  std::map<int, std::vector<std::pair<int, glm::vec3> > > map_cameras;

  std::map<int, int> map_counts;
  
  // === Draw Map3D ===
  std::vector<glm::vec3> glm_points;
  for (auto& wp: map) {
    glm::vec3 v(wp.pt.x, wp.pt.y, wp.pt.z);
    glm_points.push_back(v);

    // Counts points view nums
    int n = wp.views.size();
    auto mc_it = map_counts.find(n);
    if (mc_it != map_counts.end()) {
      mc_it->second++;
    } else {
      map_counts.insert(std::make_pair(n, 1));
    }

    for (auto& vw: wp.views) {
      int cam_id = vw.first;
      int point_id = vw.second;
      auto p = std::make_pair(point_id, v);
      auto it = map_cameras.find(cam_id);
      if (it != map_cameras.end()) {
        it->second.push_back(p);
      } else {
        std::vector<std::pair<int, glm::vec3> > vec = {p};
        map_cameras.insert(std::make_pair(cam_id, vec));
      }
    }
  }

  std::cout << "Map counts:\n";
  for (auto mc: map_counts) {
    std::cout << mc.first << " : " << mc.second << std::endl;
  }

  std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(glm_points));
  root->AddChild(points_obj);

  std::vector<std::shared_ptr<CameraObject> > camera_refs;

  // === Draw Cameras ===
  for (auto& c: map_cameras) {
    
    std::cout << "Camera: " << c.first << ": points = " << c.second.size() << std::endl;
    // PrintVec(" => ", c.second);

    const int& cam_id = c.first;
    cv::Mat co_img = images[cam_id].clone();


    std::vector<cv::KeyPoint> kpoints;
    std::vector<glm::vec3> points_3d;
    for (size_t i = 0; i < c.second.size(); ++i) {
      kpoints.push_back(image_features[cam_id].keypoints[c.second[i].first]);
      points_3d.push_back(c.second[i].second);
    }
    cv::Mat img_points;
    DrawKeypointsWithResize(co_img, kpoints, img_points, 0.3);

    
    std::shared_ptr<CameraObject> co(
        new CameraObject(cameras[cam_id].intr, kImageWidth, kImageHeight));
    co->SetTag(TAG_CAMERA_OBJECT);
    co->SetImageTransparency(true);
    // co->SetTranslation(glm::vec3(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5]));
    co->SetTranslation(cameras[cam_id].translation);
    // co->SetRotation(im_data2.coords[0], im_data2.coords[1], im_data2.coords[2]);
    co->SetRotation(cameras[cam_id].rotation_angles[0], cameras[cam_id].rotation_angles[1], cameras[cam_id].rotation_angles[2]);
    // co->SetImage(img2);
    // co->SetImage(img2_points);
    co->SetImage(img_points);
    co->SetImageAlpha(0.99);
    cameras1->AddChild(co);

    camera_refs.push_back(co);

    // co->AddProjectedPoints(glm_points);
    co->AddProjectedPoints(points_3d);
  }

  root->AddChild(cameras1);
  // root->AddChild(cameras2);

  // std::cout << "kpts_count = " << kpts_count << std::endl;


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
  int current_camera_id = 0;
  double last_camera_change = 0;  
  auto change_camera = [&cameras,
                        &camera] (int camera_id) {
    std::cout << "camera_id = " << camera_id << std::endl;
    camera->SetOrigin(glm::vec3(cameras[camera_id].translation));
    // camera->SetRotation(cameras[camera_id].rotation_angles[0],
    //                     cameras[camera_id].rotation_angles[1],
    //                     cameras[camera_id].rotation_angles[2]);
  };

  change_camera(current_camera_id);

  gl_window.AddProcessInput(GLFW_KEY_J, [&current_camera_id,
      &last_camera_change, &cameras, &gl_window, &used_views,
      &change_camera] (float dt) {
    const double key_rate = 0.2;
    double now = gl_window.GetTime();
    if (now - last_camera_change < key_rate) return;
    last_camera_change = now;
    // std::cout << "AddProcessInput!! == J = " << dt 
    //           << ", time = " << gl_window.GetTime() << std::endl;
    do {
      current_camera_id = ++current_camera_id % cameras.size();
    } while (!used_views.count(current_camera_id));

    change_camera(current_camera_id);
  });

  gl_window.AddProcessInput(GLFW_KEY_K, [&current_camera_id,
      &last_camera_change, &cameras, &gl_window, &used_views,
      &change_camera] (float dt) {
    const double key_rate = 0.2;
    double now = gl_window.GetTime();
    if (now - last_camera_change < key_rate) return;
    last_camera_change = now;
    // std::cout << "AddProcessInput!! == K = " << dt 
    //           << ", time = " << gl_window.GetTime() << std::endl;
    do {
      current_camera_id = (current_camera_id + cameras.size() - 1) % cameras.size();
    } while (!used_views.count(current_camera_id));

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


  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    // ====================== Render =====================
    renderer->Draw(floor_obj);

    renderer->Draw(root, true);


    gl_window.RunLoop();
  }


  return EXIT_SUCCESS;


}