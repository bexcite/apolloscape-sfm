// Copyright Pavlo 2018

#include <iostream>
#include <memory>
#include <iomanip>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

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

  std::vector<ImageData> camera1_poses = ReadCameraPoses(camera1_path);
  std::vector<ImageData> camera2_poses = ReadCameraPoses(camera2_path);

  std::cout << "sizes 1 = " << camera1_poses.size() << std::endl;
  std::cout << "sizes 2 = " << camera2_poses.size() << std::endl;

  std::cout << "Camera Poses 1: " << camera1_poses[1];
  std::cout << "Camera Poses 2: " << camera2_poses[1];


  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(-3.0f, 0.0f, 1.5f));
  camera->SetScale(100.0f);

  std::cout << "Hello show camera" << std::endl;

  GLWindow gl_window("OpenGL: Edit Space", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));

  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(100.0, 60));

  /*
  std::shared_ptr<ColorObject> camera_obj(
      ObjectFactory::CreateCameraFrustum());
  camera_obj->SetTranslation(glm::vec3(camera1_poses[110].coords[3], camera1_poses[110].coords[4], camera1_poses[110].coords[5]));
  // camera_obj->SetRotation(-1.7889f, 0.0250f, -1.4811f);
  camera_obj->SetRotation(camera1_poses[110].coords[0], camera1_poses[110].coords[1], camera1_poses[110].coords[2]);
  */

  camera->SetOrigin(glm::vec3(camera1_poses[110].coords[3], camera1_poses[110].coords[4], camera1_poses[110].coords[5]));
  camera->SetRotation(camera1_poses[110].coords[0], camera1_poses[110].coords[1], camera1_poses[110].coords[2]);

  // std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(1.0f));  

  // Create camera_objects
  std::vector<std::shared_ptr<ColorObject> > camera_objects1;
  std::vector<std::shared_ptr<ColorObject> > camera_objects2;

  for (const ImageData& im_data : camera1_poses) {
    std::shared_ptr<ColorObject> co(
      ObjectFactory::CreateCameraFrustum());
    co->SetScale(glm::vec3(camera->GetImageWidth()/camera->GetImageHeight(), 1.0f, 1.0f));
    co->SetTranslation(glm::vec3(im_data.coords[3], im_data.coords[4], im_data.coords[5]));
    co->SetRotation(im_data.coords[0], im_data.coords[1], im_data.coords[2]);
    // co->AddChild(axes_obj);
    camera_objects1.emplace_back(co);
  }

  for (const ImageData& im_data : camera2_poses) {
    std::shared_ptr<ColorObject> co(
      ObjectFactory::CreateCameraFrustum());
    co->SetScale(glm::vec3(camera->GetImageWidth()/camera->GetImageHeight(), 1.0f, 1.0f));
    co->SetTranslation(glm::vec3(im_data.coords[3], im_data.coords[4], im_data.coords[5]));
    co->SetRotation(im_data.coords[0], im_data.coords[1], im_data.coords[2]);
    camera_objects2.emplace_back(co);
  }

  std::shared_ptr<DObject> origin_axes_obj(ObjectFactory::CreateAxes(100.0f));

  // std::shared_ptr<ColorObject> zero_cube_obj(ObjectFactory::CreateCube());
  // zero_cube_obj->SetScale(glm::vec3(100.0f, 100.0f, 100.0f));

  // std::shared_ptr<ColorObject> x_cube_obj(ObjectFactory::CreateCube());
  // x_cube_obj->SetColor({0.8, 0.1, 0.1, 1.0});
  // x_cube_obj->SetScale(glm::vec3(100.0f, 100.0f, 100.0f));
  // x_cube_obj->SetTranslation(glm::vec3(500.0f, 0.0f, 0.0f));

  // std::shared_ptr<ColorObject> y_cube_obj(ObjectFactory::CreateCube());
  // y_cube_obj->SetColor({0.1, 0.8, 0.1, 1.0});
  // y_cube_obj->SetScale(glm::vec3(100.0f, 100.0f, 100.0f));
  // y_cube_obj->SetTranslation(glm::vec3(0.0f, 500.0f, 0.0f));

  // std::shared_ptr<ColorObject> z_cube_obj(ObjectFactory::CreateCube());
  // z_cube_obj->SetColor({0.1, 0.1, 0.8, 1.0});
  // z_cube_obj->SetScale(glm::vec3(100.0f, 100.0f, 100.0f));
  // z_cube_obj->SetTranslation(glm::vec3(0.0f, 0.0f, 500.0f));

  std::shared_ptr<ModelObject> debug_cube_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/debug_cube/debug_cube.obj"));
  debug_cube_obj->SetScale(glm::vec3(100.0f));
  debug_cube_obj->SetTranslation(glm::vec3(300.0f, 300.0f, 300.0f));


  // std::cout << "Floor = " << floor_obj << std::endl;
  // std::cout << "Camera = " << camera_obj << std::endl;
  // std::cout << "Zero Cube = " << zero_cube_obj << std::endl;
  // std::cout << "X Cube = " << x_cube_obj << std::endl;
  // std::cout << "Y Cube = " << y_cube_obj << std::endl;
  // std::cout << "Z Cube = " << z_cube_obj << std::endl;
  // std::cout << "Debug Cube = " << debug_cube_obj << std::endl;


  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    /* ====================== Render ===================== */
    renderer->Draw(floor_obj);
    // renderer->Draw(camera_obj);
    // renderer->Draw(zero_cube_obj);
    renderer->Draw(debug_cube_obj);

    // renderer->Draw(x_cube_obj);
    // renderer->Draw(y_cube_obj);
    // renderer->Draw(z_cube_obj);

    renderer->Draw(origin_axes_obj);

    for (const auto& co : camera_objects1) {
      renderer->Draw(co);
    }

    for (const auto& co : camera_objects2) {
      renderer->Draw(co);
    }
  
    gl_window.RunLoop();
  }

  return EXIT_SUCCESS;

}
