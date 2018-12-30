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
  

  GLWindow gl_window("OpenGL: Two Cameras", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);  

  
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
  camera1_obj->SetRotation(static_cast<float>(M_PI_2), 0.0f, static_cast<float>(M_PI_2));
  // camera1_obj->SetImage(image1_path.string());
  camera1_obj->SetImage(img1);
  camera1_obj->SetImageTransparency(true);
  camera1_obj->SetImageAlpha(0.9);
  root->AddChild(camera1_obj);

  std::shared_ptr<CameraObject> camera2_obj(new CameraObject(camera_intr));
  camera2_obj->SetTranslation(glm::vec3(3.0f, -0.7f, 1.0f));
  camera2_obj->SetRotation(static_cast<float>(M_PI_2), 0.0f, static_cast<float>(M_PI_2));
  // camera2_obj->SetImage(image2_path.string());
  camera2_obj->SetImage(img2);
  camera2_obj->SetImageTransparency(true);
  camera2_obj->SetImageAlpha(0.9);
  root->AddChild(camera2_obj);




  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    /* ====================== Render ===================== */
    renderer->Draw(floor_obj);

    renderer->Draw(root, true);

    // renderer->Draw(camera_obj, true);

    gl_window.RunLoop();
  }

  return EXIT_SUCCESS;


}
