// Copyright Pavlo 2018

#include <algorithm>
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

const float kGlobalScale = 100.0f;

#define TAG_CAMERA_OBJECT 10


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

  float image_width = 2452;
  float image_height = 2056;

  CameraIntrinsics camera1_intr, camera2_intr;
  camera1_intr.cx = 1244.386581025;
  camera1_intr.cy = 1013.145997723;
  camera1_intr.fx = 1450.317230113;
  camera1_intr.fy = 1451.184836113;

  camera2_intr.cx = 1250.940749515;
  camera2_intr.cy = 1013.259732772;
  camera2_intr.fx = 1448.572928508;
  camera2_intr.fy = 1449.555907804;

  std::vector<ImageData> camera1_poses = ReadCameraPoses(camera1_path);
  std::vector<ImageData> camera2_poses = ReadCameraPoses(camera2_path);

  std::cout << "sizes 1 = " << camera1_poses.size() << std::endl;
  std::cout << "sizes 2 = " << camera2_poses.size() << std::endl;

  std::cout << "Camera Poses 1: " << camera1_poses[1];
  std::cout << "Camera Poses 2: " << camera2_poses[1];


  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(-3.0f, 0.0f, 1.5f));
  camera->SetIntrinsics(camera1_intr);
  camera->SetScale(kGlobalScale);

  // Normalize Intrinsics
  camera1_intr.cx /= image_width;
  camera1_intr.cy /= image_height;
  camera1_intr.fx /= image_width;
  camera1_intr.fy /= image_height;

  camera2_intr.cx /= image_width;
  camera2_intr.cy /= image_height;
  camera2_intr.fx /= image_width;
  camera2_intr.fy /= image_height;

  // CameraIntrinsics camera_intr = camera->GetCameraIntrinsics();

  // float fx = camera->GetFx() / camera->GetImageWidth();
  // float fy = camera->GetFy() / camera->GetImageHeight();
  // float f = std::max(fx, fy);

  std::cout << "Hello show camera" << std::endl;

  GLWindow gl_window("OpenGL: Edit Space", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));

  std::shared_ptr<ColorObject> floor_obj(ObjectFactory::CreateFloor(kGlobalScale, 60));

  /*
  std::shared_ptr<ColorObject> camera_obj(
      ObjectFactory::CreateCameraFrustum());
  camera_obj->SetTranslation(glm::vec3(camera1_poses[110].coords[3], camera1_poses[110].coords[4], camera1_poses[110].coords[5]));
  // camera_obj->SetRotation(-1.7889f, 0.0250f, -1.4811f);
  camera_obj->SetRotation(camera1_poses[110].coords[0], camera1_poses[110].coords[1], camera1_poses[110].coords[2]);
  */

  // camera->SetOrigin(glm::vec3(camera1_poses[110].coords[3], camera1_poses[110].coords[4], camera1_poses[110].coords[5]));
  // camera->SetRotation(camera1_poses[110].coords[0], camera1_poses[110].coords[1], camera1_poses[110].coords[2]);

  int p_camera_pose = 24; // 24
  int p_camera_start = 22; //22
  int p_camera_finish = 25; //25

  const ImageData& camera_origin_data = camera1_poses[p_camera_pose];

  camera->SetOrigin(glm::vec3(camera_origin_data.coords[3], camera_origin_data.coords[4], camera_origin_data.coords[5]));
  camera->SetRotation(camera_origin_data.coords[0], camera_origin_data.coords[1], camera_origin_data.coords[2]);
  // camera->SetOrigin(glm::vec3(3.0f, 0.0f, 0.0f));

  float width_ratio = camera->GetImageWidth()/camera->GetImageHeight();

  std::shared_ptr<DObject> axes_obj(ObjectFactory::CreateAxes(1.0f));

  std::shared_ptr<DObject> cameras1(new DObject());
  // cameras1->SetTranslation(glm::vec3(-100.0f));

  std::shared_ptr<DObject> cameras2(new DObject());

  std::cout << "Loading camera 1: " << std::flush;
  for (int i = p_camera_start; i < camera1_poses.size(); ++i) {
    if (i == p_camera_finish) break;
    const ImageData& im_data = camera1_poses[i];

    // TODO: width ratio for camera obj is not good, it should be defined by image probably
    std::shared_ptr<CameraObject> co(new CameraObject(width_ratio, camera1_intr));
    co->SetTag(TAG_CAMERA_OBJECT);
    co->SetTranslation(glm::vec3(im_data.coords[3], im_data.coords[4], im_data.coords[5]));
    co->SetRotation(im_data.coords[0], im_data.coords[1], im_data.coords[2]);

    fs::path image_path = camera1_image_path / fs::path(im_data.filename);
    co->SetImage(image_path.string());

    // Add points of previous camera
    if (i > 0) {
      const ImageData& im_data1 = camera1_poses[i-1];
      const ImageData& im_data2 = camera2_poses[i-1];

      std::vector<glm::vec3> points = {
        glm::vec3(im_data1.coords[3], im_data1.coords[4], im_data1.coords[5]),
        glm::vec3(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5])
      };

      std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(points));
      cameras1->AddChild(points_obj);

      co->AddProjectedPoints(points);
    }

    // co->AddChild(axes_obj);
    cameras1->AddChild(co);

    std::cout << i << ". " << std::flush;

    std::cout << "co = " << co << std::endl;
  }

  std::cout << std::endl << "Loading camera 2: " << std::flush;
  for (int i = p_camera_start; i < camera2_poses.size(); ++i) {
    if (i == p_camera_finish) break;
    const ImageData& im_data = camera2_poses[i];

    // TODO: width ratio for camera obj is not good, it should be defined by image probably
    std::shared_ptr<CameraObject> co(new CameraObject(width_ratio, camera2_intr));
    co->SetTag(TAG_CAMERA_OBJECT);
    co->SetTranslation(glm::vec3(im_data.coords[3], im_data.coords[4], im_data.coords[5]));
    co->SetRotation(im_data.coords[0], im_data.coords[1], im_data.coords[2]);

    fs::path image_path = camera2_image_path / fs::path(im_data.filename);
    co->SetImage(image_path.string());

    // Add points of previous camera
    if (i > 0) {
      const ImageData& im_data1 = camera1_poses[i-1];
      const ImageData& im_data2 = camera2_poses[i-1];

      std::vector<glm::vec3> points = {
        glm::vec3(im_data1.coords[3], im_data1.coords[4], im_data1.coords[5]),
        glm::vec3(im_data2.coords[3], im_data2.coords[4], im_data2.coords[5])
      };

      std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(points));
      cameras1->AddChild(points_obj);

      co->AddProjectedPoints(points);
    }

    // co->AddChild(axes_obj);
    cameras2->AddChild(co);

    std::cout << i << ". " << std::flush;
  }

  std::shared_ptr<DObject> origin_axes_obj(ObjectFactory::CreateAxes(kGlobalScale));


  std::shared_ptr<ModelObject> debug_cube_obj(
      ObjectFactory::CreateModelObject(
          "../data/objects/debug_cube/debug_cube.obj"));
  debug_cube_obj->SetScale(glm::vec3(kGlobalScale));
  debug_cube_obj->SetTranslation(glm::vec3(3.0f * kGlobalScale, 3.0f * kGlobalScale, 3.0f * kGlobalScale));


  // std::shared_ptr<DObject> full_camera_obj(new DObject());
  // std::shared_ptr<ColorObject> camera_obj(
  //     ObjectFactory::CreateCameraFrustum());
  // camera_obj->SetScale(glm::vec3(camera->GetImageWidth()/camera->GetImageHeight(), 1.0f, 1.0f));
  // camera_obj->SetTranslation(glm::vec3(0.0f, 0.0f, 0.0f));
  // full_camera_obj->AddChild(camera_obj);

  // std::shared_ptr<ImageObject> image_obj(ObjectFactory::CreateImage());
  // image_obj->SetScale(glm::vec3(camera->GetImageWidth()/camera->GetImageHeight(), 1.0f, 1.0f));
  // image_obj->SetTranslation(glm::vec3(0.0f, 0.0f, -1.0f));

  fs::path image_path = camera1_image_path / fs::path(camera1_poses[0].filename);
  // image_obj->SetImage(image_path.string(), false);
  // full_camera_obj->AddChild(image_obj);

  // full_camera_obj->SetTranslation(glm::vec3(3.0f, 0.0f, 0.0f));

  // full_camera_obj->SetTranslation(glm::vec3(camera1_poses[0].coords[3], camera1_poses[0].coords[4], camera1_poses[0].coords[5]));
  // full_camera_obj->SetRotation(camera1_poses[0].coords[0], camera1_poses[0].coords[1], camera1_poses[0].coords[2]);


  

  std::shared_ptr<CameraObject> full_camera(new CameraObject(width_ratio));
  full_camera->SetImage(image_path.string());
  full_camera->SetRotation(camera1_poses[0].coords[0], camera1_poses[0].coords[1], camera1_poses[0].coords[2]);
  full_camera->SetTranslation(glm::vec3(3.0f, 0.0f, 0.0f));


  // std::cout << "Floor = " << floor_obj << std::endl;
  // std::cout << "Camera = " << camera_obj << std::endl;
  // std::cout << "Debug Cube = " << debug_cube_obj << std::endl;

  // int cntr = 0;


  float alpha = 0.0f;

  auto change_alpha = [&cameras1, &cameras2, &alpha]() {
    auto set_alpha = [&alpha](std::shared_ptr<DObject> obj) {
      std::shared_ptr<CameraObject> co = std::static_pointer_cast<CameraObject>(obj);
      co->SetImageAlpha(alpha);
    };
    cameras1->Apply(TAG_CAMERA_OBJECT, set_alpha);
    cameras2->Apply(TAG_CAMERA_OBJECT, set_alpha);
  };


  gl_window.AddProcessInput(GLFW_KEY_Z, [&alpha, &change_alpha](float dt) {
    alpha = std::max(alpha - 0.9f * dt, 0.0f);
    // std::cout << "AddProcessInput!! == Z = " << alpha << std::endl;
    change_alpha();
  });

  gl_window.AddProcessInput(GLFW_KEY_X, [&alpha, &change_alpha](float dt) {
    alpha = std::min(alpha + 0.9f * dt, 1.0f);
    // std::cout << "AddProcessInput!! == X = " << alpha << std::endl;
    change_alpha();
  });

  

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    /* ====================== Render ===================== */
    renderer->Draw(floor_obj);
    // renderer->Draw(camera_obj);
    renderer->Draw(debug_cube_obj);

    renderer->Draw(origin_axes_obj);

    renderer->Draw(cameras1);
    renderer->Draw(cameras2);

    // float a = sin(2 * M_PI * (cntr) / 5000.0f);
    // auto set_alpha = [=](std::shared_ptr<DObject> obj) {
    //   std::shared_ptr<CameraObject> co = std::static_pointer_cast<CameraObject>(obj);
    //   co->SetImageAlpha(a);
    // };
    // cameras1->Apply(TAG_CAMERA_OBJECT, set_alpha);

    /*
    std::list<std::shared_ptr<DObject> > chlds = cameras1->GetChildren();
    // std::shared_ptr<CameraObject> c1 = std::static_pointer_cast<CameraObject>(chlds.front());
    // std::cout << "chlds.size = " << chlds.size() << std::endl;
    // std::cout << "c1 = " << c1 << std::endl;
    for (auto it = chlds.begin(); it != chlds.end(); ++it) {
      std::shared_ptr<DObject> dobj = *it;
      if (dobj->GetTag() == TAG_CAMERA_OBJECT) {
        float a = sin(2 * M_PI * (cntr) / 5000.0f);

        auto set_alpha = [=](std::shared_ptr<DObject> obj) {
          std::shared_ptr<CameraObject> co = std::static_pointer_cast<CameraObject>(obj);
          co->SetImageAlpha(a);
        };

        set_alpha(dobj);


        // std::shared_ptr<CameraObject> co = std::static_pointer_cast<CameraObject>(dobj);
        // std::cout << "dobj = " << co << std::endl;
        // float alpha = 
        // co->SetImageAlpha(sin(2 * M_PI * (cntr) / 5000.0f)); 
      }
    }
    */

    // ++cntr;

    // std::cout << "cntr = " << cntr << std::endl;

    // c1->SetImageAlpha((cntr % 1000) / 1000.0f);

    // renderer->Draw(image_obj);

    // renderer->Draw(full_camera_obj);

    renderer->Draw(full_camera);
  
    gl_window.RunLoop();
  }

  return EXIT_SUCCESS;

}
