// Copyright Pavlo 2018

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


const int kWindowWidth = 1226/2;
const int kWindowHeight = 1028/2;

const char kApolloDatasetPath[] = "../data/apolloscape";

const char kRoadId[] = "zpark-sample";
const char kRecordId[] = "Record001";

const char kCamera1PoseFile[] = "Camera_1.txt";
const char kCamera2PoseFile[] = "Camera_2.txt";



int main(int argc, char* argv[]) {

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


  ImageData& im_data = camera1_poses[0];

  fs::path image_path = camera1_image_path / fs::path(im_data.filename);
  std::cout << "image_path = " << image_path << std::endl;


  cv::Mat img = cv::imread(image_path.string().c_str());
  std::cout << "img = " << img.size << std::endl;
  std::cout << "img.step = " << img.step << std::endl;
  std::cout << "img.step[0] = " << img.step[0] << std::endl;
  std::cout << "img.step[1] = " << img.step[1] << std::endl;
  std::cout << "img.elemSize = " << img.elemSize() << std::endl;
  glm::vec3 im_size_vec = glm::vec3(img.size().width, img.size().height, 1.0f);
  std::cout << "im_size_vec = " << glm::to_string(im_size_vec) << std::endl;


  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(-3.0f, 0.0f, 1.5f));

  std::cout << "Hello edit space" << std::endl;

  float width_ratio = camera->GetImageWidth()/camera->GetImageHeight();
  CameraIntrinsics camera_intr = camera->GetCameraIntrinsics();

  // float fx = camera->GetFx() / camera->GetImageWidth();
  // float fy = camera->GetFy() / camera->GetImageHeight();
  std::cout << "fx = " << camera_intr.fx << std::endl;
  std::cout << "fy = " << camera_intr.fy << std::endl;
  float f = std::max(camera_intr.fx, camera_intr.fy);

  glm::vec4 v(1.0f);
  std::cout << "v = " << v << std::endl;

  GLWindow gl_window("OpenGL: Edit Space", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));

  std::shared_ptr<DObject> root = std::make_shared<DObject>();

  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(1.0, 50));

  std::shared_ptr<CameraObject> camera_obj(new CameraObject(width_ratio, camera_intr));
  camera_obj->SetTranslation(glm::vec3(3.0f, 0.0f, 1.0f));
  // camera_obj->SetRotation(-1.7889f, 0.0250f, 0.0f);
  // camera_obj->SetRotation(-1.7889f, 0.0250f, -1.4811f);
  camera_obj->SetRotation(static_cast<float>(M_PI_2), 0.0f, static_cast<float>(M_PI_2));
  camera_obj->SetImage(image_path.string());
  // camera_obj->SetImage(img);
  camera_obj->SetImageTransparency(true);
  camera_obj->SetImageAlpha(0.3);
  // root->AddChild(camera_obj);

  // Main Camera pos/rot
  // camera->SetPosition(glm::vec3(3.0f, 0.0f, 1.0f));
  camera->SetOrigin(glm::vec3(3.0f, 0.0f, 1.0f));
  // camera->SetRotation(-1.7889f, 0.0250f, -1.4811f);
  // camera->SetRotation(0.0f, 0.0f, 0.0f);
  // camera->SetDirection(glm::vec3(0.0f, 0.0f, 1.0f));

  camera_intr = camera_obj->GetCameraIntrinsics();
  glm::mat3 camera_matrix = camera_intr.GetCameraMatrix();
  std::cout << "wr = " << camera_intr.wr << std::endl;
  std::cout << "Camera Intrinsic Matrix: " << camera_matrix << std::endl;

  glm::mat3 rotation_matrix(camera_obj->GetRotation());
  glm::vec3 translation(camera_obj->GetTranslation());
  std::cout << "Rotation Matrix: " << rotation_matrix << std::endl;
  std::cout << "Translation: " << translation << std::endl;

  glm::vec3 vec1(1.0, 0.0, 0.0);
  glm::vec3 vec2(0.0, 0.0, 1.0);
  glm::vec3 vec3(4.0, 0.0, 1.0);
  std::cout << "Rotated Vector " << glm::to_string(vec1) << " back : " << (glm::transpose(rotation_matrix) * vec1) << std::endl;
  std::cout << "Rotated Vector " << glm::to_string(vec2) << " back : " << (glm::transpose(rotation_matrix) * vec2) << std::endl;

  glm::mat4x3 proj(glm::transpose(rotation_matrix));
  
  proj[3] -= glm::transpose(rotation_matrix) * translation;
  std::cout << "Proj 4x3 test: " << glm::to_string(proj) << std::endl;
  std::cout << "Proj Vec " << glm::to_string(vec3) << " to camera : " << (proj * glm::vec4(vec3, 1.0)) << std::endl;

  glm::mat4x3 proj_translation(1.0);
  proj_translation[3] -= translation;

  std::cout << "Proj Vec 2 stage " << glm::to_string(vec3) << " to camera coord : "
      << (glm::transpose(rotation_matrix) * proj_translation * glm::vec4(vec3, 1.0)) << std::endl;

  std::cout << "Proj Vec to camera " << glm::to_string(vec3) << " : "
      << (camera_matrix * glm::transpose(rotation_matrix) * proj_translation * glm::vec4(vec3, 1.0)) << std::endl;


  float point_dist = 2.0f;
  glm::vec3 point0(7.0, 0.0, 1.0);
  glm::vec3 point1(point0[0], point0[1] - point_dist, point0[2] - point_dist); // 1st (1stX + 1stY)
  glm::vec3 point2(point0[0], point0[1] + point_dist, point0[2] - point_dist); // 2nd (2ndX + 1stY)
  glm::vec3 point3(point0[0], point0[1] - point_dist, point0[2] + point_dist); // 3rd (1stX + 2ndY)
  glm::vec3 point4(point0[0], point0[1] + point_dist, point0[2] + point_dist); // 4th (2ndX + 2ndY)


  glm::vec3 point0_cam = camera_matrix * glm::transpose(rotation_matrix) * proj_translation * glm::vec4(point0, 1.0);
  glm::vec3 point1_cam = camera_matrix * glm::transpose(rotation_matrix) * proj_translation * glm::vec4(point1, 1.0);
  glm::vec3 point2_cam = camera_matrix * glm::transpose(rotation_matrix) * proj_translation * glm::vec4(point2, 1.0);
  glm::vec3 point3_cam = camera_matrix * glm::transpose(rotation_matrix) * proj_translation * glm::vec4(point3, 1.0);
  glm::vec3 point4_cam = camera_matrix * glm::transpose(rotation_matrix) * proj_translation * glm::vec4(point4, 1.0);

  point0_cam /= point0_cam[2];
  point1_cam /= point1_cam[2];
  point2_cam /= point2_cam[2];
  point3_cam /= point3_cam[2];
  point4_cam /= point4_cam[2];

  std::cout << "Proj Point 0 to camera " << glm::to_string(point0) << " : "
      << (point0_cam) << std::endl;
  std::cout << "Proj Point 1 to camera " << glm::to_string(point1) << " : "
      << (point1_cam) << std::endl;
  std::cout << "Proj Point 2 to camera " << glm::to_string(point2) << " : "
      << (point2_cam) << std::endl;
  std::cout << "Proj Point 3 to camera " << glm::to_string(point3) << " : "
      << (point3_cam) << std::endl;
  std::cout << "Proj Point 4 to camera " << glm::to_string(point4) << " : "
      << (point4_cam) << std::endl;

  std::cout << "Point 0 : " << glm::to_string(point0_cam * im_size_vec) << " pixels" << std::endl;
  std::cout << "Point 1 : " << glm::to_string(point1_cam * im_size_vec) << " pixels" << std::endl;
  std::cout << "Point 2 : " << glm::to_string(point2_cam * im_size_vec) << " pixels" << std::endl;
  std::cout << "Point 3 : " << glm::to_string(point3_cam * im_size_vec) << " pixels" << std::endl;
  std::cout << "Point 4 : " << glm::to_string(point4_cam * im_size_vec) << " pixels" << std::endl;

  cv::circle(img, cv::Point(point0_cam[0] * im_size_vec[0], point0_cam[1] * im_size_vec[1]), 20, cv::Scalar(0, 0, 255), 3);
  cv::line(img,
      cv::Point(point0_cam[0] * im_size_vec[0], point0_cam[1] * im_size_vec[1]),
      cv::Point(point1_cam[0] * im_size_vec[0], point1_cam[1] * im_size_vec[1]),
      cv::Scalar(255, 0, 255),
      3);
  cv::rectangle(img,
      cv::Point(point0_cam[0] * im_size_vec[0], point0_cam[1] * im_size_vec[1]),
      cv::Point(point2_cam[0] * im_size_vec[0], point2_cam[1] * im_size_vec[1]),
      cv::Scalar(0, 255, 255),
      cv::FILLED);
  std::stringstream ss;
  ss << point3_cam[1] * im_size_vec[1];
  cv::putText(img,
      ss.str(),
      cv::Point(point3_cam[0] * im_size_vec[0], point3_cam[1] * im_size_vec[1]),
      0,
      2.0,
      cv::Scalar(255, 255, 255),
      5);
  camera_obj->SetImage(img);

  std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(1.0f));
  // camera_obj->AddChild(axes_obj);
  // root->AddChild(axes_obj);


  std::vector<glm::vec3> points = {glm::vec3(0.0f), glm::vec3(1.0f), glm::vec3(2.0f), glm::vec3(3.0f), glm::vec3(4.0f)};
  for (int i = 0; i < points.size(); ++i) {
    points[i] += glm::vec3(10.0f, 0.0f, 1.0f);
  }
  
  // std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(points));
  // points_obj->SetTranslation(glm::vec3(10.0f, 0.0f, 1.0f));
  // points_obj->AddChild(axes_obj);
  // root->AddChild(points_obj);
  // camera_obj->AddProjectedPoints(points);

  std::vector<glm::vec3> points_cam = { point0, point1, point2, point3, point4 };
  std::shared_ptr<DObject> points_cam_obj(ObjectFactory::CreatePoints(points_cam));
  root->AddChild(points_cam_obj);

  std::vector<glm::vec3> points_camp = {
    point0_cam / point0_cam[2], 
    point1_cam / point1_cam[2], 
    point2_cam / point2_cam[2], 
    point3_cam / point3_cam[2], 
    point4_cam / point4_cam[2]
  };
  for (int i = 0; i < points_camp.size(); ++i) {
    points_camp[i] = rotation_matrix * (points_camp[i] - glm::vec3(camera_matrix[0][0], camera_matrix[1][1], 0.0f)) + translation;
  }
  std::shared_ptr<DObject> points_camp_obj(ObjectFactory::CreatePoints(points_camp));
  root->AddChild(points_camp_obj);


  root->AddChild(axes_obj);




  std::shared_ptr<ModelObject> debug_cube_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/debug_cube/debug_cube.obj"));
  debug_cube_obj->SetScale(glm::vec3(1.0f));
  debug_cube_obj->SetTranslation(glm::vec3(3.0f, 3.0f, 3.0f));
  // root->AddChild(debug_cube_obj);



/*
  std::shared_ptr<ModelObject> nanosuit_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/nanosuit/nanosuit.obj"));
  nanosuit_obj->SetScale(glm::vec3(0.2f));
  nanosuit_obj->SetTranslation(glm::vec3(5.0f, 2.0f, 0.0f));

  std::shared_ptr<ModelObject> cyborg_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/cyborg/cyborg.obj"));
  cyborg_obj->SetScale(glm::vec3(0.8f));
  cyborg_obj->SetTranslation(glm::vec3(6.0f, 1.0f, 0.0f));

  std::shared_ptr<ModelObject> planet_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/planet/planet.obj"));
  // planet_obj->SetScale(glm::vec3(0.8f));
  planet_obj->SetTranslation(glm::vec3(10.0f, -5.0f, 6.0f));

  std::shared_ptr<ModelObject> rock_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/rock/rock.obj"));
  rock_obj->SetScale(glm::vec3(0.3f));
  rock_obj->SetTranslation(glm::vec3(5.0f, -3.0f, 0.0f));
  */

//   std::cout << "Floor = " << floor_obj << std::endl;
//   std::cout << "Camera = " << camera_obj << std::endl;
  // std::cout << "Debug Cube = " << debug_cube_obj << std::endl;

  /*
  std::cout << "Nanosuit = " << nanosuit_obj << std::endl;
  std::cout << "Cyborg = " << cyborg_obj << std::endl;
  std::cout << "Planet = " << planet_obj << std::endl;
  std::cout << "Rock = " << rock_obj << std::endl;
  */

 root->AddChild(camera_obj);
 root->AddChild(debug_cube_obj);
 

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    /* ====================== Render ===================== */
    renderer->Draw(floor_obj);

    // renderer->Draw(debug_cube_obj, true);

    renderer->Draw(root, true);

    // renderer->Draw(camera_obj, true);
    
    

    

    

    // camera->Print();

    // renderer->Draw(axes);

    /*
    renderer->Draw(nanosuit_obj);
    renderer->Draw(cyborg_obj);
    renderer->Draw(planet_obj);
    renderer->Draw(rock_obj);
    */

    gl_window.RunLoop();
  }

  return EXIT_SUCCESS;

}
