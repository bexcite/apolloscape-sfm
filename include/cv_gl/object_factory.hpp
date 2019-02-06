// Copyright Pavlo 2018
#ifndef CV_GL_OBJECT_FACTORY_H_
#define CV_GL_OBJECT_FACTORY_H_

#include <map>
#include <iostream>

// #include <glm/gtc/matrix_transform.hpp>

// #include <cv_gl/dobject.h>
#include "cv_gl/mesh.h"
#include "cv_gl/shader.h"
#include "cv_gl/sfm_common.h"

std::vector<Vertex> CalcNormalsFromVertInd(const std::vector<Vertex> &vertices, const std::vector<uint> &indices) {
  
  int size = indices.size();

  std::vector<Vertex> nv(3 * (size / 3));

  for (int i = 0; i < size - 2; i += 3) {
    glm::vec3 vec1 = vertices[indices[i + 1]].position - vertices[indices[i]].position;
    glm::vec3 vec2 = vertices[indices[i + 2]].position - vertices[indices[i]].position;
    glm::vec3 normal = glm::normalize(glm::cross(vec1, vec2));
    nv[i] = vertices[indices[i]];
    nv[i].normal = normal;
    nv[i + 1] = vertices[indices[i + 1]];
    nv[i + 1].normal = normal;
    nv[i + 2] = vertices[indices[i + 2]];
    nv[i + 2].normal = normal;
  }


  return nv;
}

class MeshFactory {

public:

  static std::shared_ptr<Mesh> CreateGrid(const double step_size,
      const unsigned int line_num) {
    const float shift_x = - (step_size * line_num) / 2;
    const float shift_z = shift_x;
    const float far_edge = step_size * line_num;
    std::vector<Vertex> vertices((line_num + 1) * 4);
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;

    // Fill vertices
    for (unsigned int i = 0; i <= line_num; ++i) {
      Vertex v1 = {glm::vec3(step_size * i + shift_x, 0.0f, 0.0f + shift_z)};
      Vertex v2 = {glm::vec3(step_size * i + shift_x, 0.0f, far_edge + shift_z)};
      vertices.emplace_back(v1);
      vertices.emplace_back(v2);
      v1 = {glm::vec3(0.0f + shift_x, 0.0f, step_size * i + shift_z)};
      v2 = {glm::vec3(far_edge + shift_x, 0.0f, step_size * i + shift_z)};
      vertices.emplace_back(v1);
      vertices.emplace_back(v2);
    }

    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::LINES);
    return mesh;
  }


  static std::shared_ptr<Mesh> CreateCameraFrustum() {
    std::vector<Texture> textures;
    std::vector<Vertex> vertices = {
      // {glm::vec3(-1.0f, 0.0f, 1.0f)},  // 0
      // {glm::vec3(1.0f, 0.0f, 1.0f)},   // 1
      // {glm::vec3(1.0f, 0.0f, -1.0f)},  // 2
      // {glm::vec3(-1.0f, 0.0f, -1.0f)}, // 3
      {glm::vec3(-0.5f, 0.0f, 0.5f)},  // 0
      {glm::vec3(0.5f, 0.0f, 0.5f)},   // 1
      {glm::vec3(0.5f, 0.0f, -0.5f)},  // 2
      {glm::vec3(-0.5f, 0.0f, -0.5f)}, // 3
      {glm::vec3(0.0f, 1.0f, 0.0f)}   // 4
    };
    std::vector<unsigned int> indices = {
      0, 1,
      1, 2,
      2, 3,
      3, 0,
      0, 4,
      1, 4,
      2, 4,
      3, 4
    };
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::LINES);
    return mesh;
  }

  static std::shared_ptr<Mesh> CreateCameraUpTriangle() {
    std::vector<Texture> textures;
    std::vector<Vertex> vertices = {
      {glm::vec3(0.0f, 0.0f, 0.5f)},  // 0
      {glm::vec3(0.0f, 0.0f, -0.5f)},   // 1
      {glm::vec3(-0.5f, 0.0f, 0.0f)}
    };
    std::vector<unsigned int> indices;
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::TRIANGLES);
    return mesh;
  }


  static std::shared_ptr<Mesh> CreateImagePlane() {
    std::vector<Texture> textures;
    std::vector<Vertex> vertices = {
      // {glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)},  // 0
      // {glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)},   // 1
      // {glm::vec3(0.5f, 0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)},  // 2
      // {glm::vec3(-0.5f, 0.5f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)} // 3

      {glm::vec3(-0.5f, 0.0f, 0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)},  // 0
      {glm::vec3(0.5f, 0.0f, 0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)},   // 1
      {glm::vec3(0.5f, 0.0f, -0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)},  // 2
      {glm::vec3(-0.5f, 0.0f, -0.5f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)} // 3

      // {glm::vec3(-0.5f, 0.0f, 0.5f)},  // 0
      // {glm::vec3(0.5f, 0.0f, 0.5f)},   // 1
      // {glm::vec3(0.5f, 0.0f, -0.5f)},  // 2
      // {glm::vec3(-0.5f, 0.0f, -0.5f)}, // 3
    };
    std::vector<unsigned int> indices = {
      0, 1, 2,
      0, 2, 3
    };
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::TRIANGLES);
    return mesh;
  }

  

  static std::shared_ptr<Mesh> CreateCube() {
    std::vector<Texture> textures;
    std::vector<Vertex> vertices = {
      // bottom
      {glm::vec3(-0.5f, -0.5f, 0.5f)},  // 0
      {glm::vec3(0.5f, -0.5f, 0.5f)},   // 1
      {glm::vec3(0.5f, -0.5f, -0.5f)},  // 2
      {glm::vec3(-0.5f, -0.5f, -0.5f)}, // 3
      // top
      {glm::vec3(-0.5f, 0.5f, 0.5f)},   // 4
      {glm::vec3(0.5f, 0.5f, 0.5f)},    // 5
      {glm::vec3(0.5f, 0.5f, -0.5f)},   // 6
      {glm::vec3(-0.5f, 0.5f, -0.5f)},  // 7
      
    };

    // std::vector<unsigned int> indices = {
    //   0, 1, 2, 
    //   0, 2, 3,
    //   4, 5, 6,
    //   4, 6, 7,
    //   0, 3, 4,
    //   4, 3, 7,
    //   0, 1, 5,
    //   0, 5, 4,
    //   3, 2, 6,
    //   3, 6, 7,
    //   1, 2, 6,
    //   1, 6, 5
    // };

    std::vector<unsigned int> indices = {
        0, 2, 1,
        0, 3, 2,
        4, 5, 6,
        4, 6, 7,
        0, 4, 3,
        4, 7, 3,
        0, 1, 5,
        0, 5, 4,
        3, 6, 2,
        3, 7, 6,
        1, 2, 6,
        1, 6, 5};

    vertices = CalcNormalsFromVertInd(vertices, indices);
    indices.clear();

    // std::cout << "-----> vertices = " << vertices << std::endl;

    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::TRIANGLES);
    return mesh;
  }

  static std::shared_ptr<Mesh> CreatePoints(std::vector<glm::vec3>& points) {
    std::vector<Texture> textures;
    std::vector<Vertex> vertices(points.size());
    for (int i = 0; i < points.size(); ++i) {
      vertices[i].position = points[i];
    }
    std::vector<unsigned int> indices;
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::POINTS);
    return mesh;
  }

  static std::shared_ptr<Mesh> CreatePoints(std::vector<Point3DColor>& points) {
    std::vector<Texture> textures;
    std::vector<Vertex> vertices(points.size());
    for (int i = 0; i < points.size(); ++i) {
      vertices[i].position = points[i].pt;
      vertices[i].color = points[i].color;
      vertices[i].color_tl = points[i].color_tl;
      vertices[i].color_tr = points[i].color_tr;
      vertices[i].color_bl = points[i].color_bl;
      vertices[i].color_br = points[i].color_br;
    }
    std::vector<unsigned int> indices;
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::POINTS);
    return mesh;
  }


};


// class CameraObject;




class ObjectFactory {

public:

  static std::map<std::string, std::shared_ptr<Shader> > shaders_;

  static std::shared_ptr<Shader> GetShader(const std::string& vertex_path,
      const std::string& fragment_path, const std::string& geometry_path = "") {
    // check existence
    auto it = shaders_.find(vertex_path + fragment_path + geometry_path);
    if (it != shaders_.end()) {
      return it->second;
    }
    // create new
    std::shared_ptr<Shader> shader = std::make_shared<Shader>(vertex_path, 
        fragment_path, geometry_path);
    shaders_.insert(std::pair<std::string, std::shared_ptr<Shader> >(
          vertex_path + fragment_path + geometry_path, shader));
    return shader;
  }

/* ================ Floor ================================*/
  static ColorObject* CreateFloor(const double step_size,
      const unsigned int line_num) {

    auto mesh = MeshFactory::CreateGrid(step_size, line_num);

    // auto shader_color = std::make_shared<Shader>(
    //   "../shaders/one.vs",
    //   "../shaders/one_color.fs");

    // auto shader_color = std::make_shared<Shader>(
    //     "../shaders/two.vs",
    //     "../shaders/two_model.fs");

    auto shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ColorObject* floor_obj =
        new ColorObject(mesh, glm::vec4(0.5f, 0.5f, 0.8f, 1.0f));
    floor_obj->SetShader(shader_color);

    return floor_obj;

  }


/* ================ Camera ================================*/
  static ColorObject* CreateCameraFrustum() {

    auto mesh = MeshFactory::CreateCameraFrustum();

    // auto shader_color = std::make_shared<Shader>(
    //   "../shaders/one.vs",
    //   "../shaders/one_color.fs");

    // auto shader_color = std::make_shared<Shader>(
    //     "../shaders/two.vs",
    //     "../shaders/two_model.fs");


    auto shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ColorObject* camera_obj =
        new ColorObject(mesh, glm::vec4(1.0f, 0.7f, 0.7f, 1.0f));
    camera_obj->SetShader(shader_color);

    // Add additional correction to start from origin
    glm::mat4 correction(1.0f);
    correction = glm::translate(correction, glm::vec3(0.0f, 0.0f, -1.0f));
    camera_obj->AddToCorrectionMatrix(correction);

    // Add Up Triangle
    auto tri_mesh = MeshFactory::CreateCameraUpTriangle();
    ColorObject* tri_up_obj =
        new ColorObject(tri_mesh, glm::vec4(0.1f, 0.1f, 0.1f, 1.0f));
    tri_up_obj->SetShader(shader_color);
    tri_up_obj->SetScale(glm::vec3(0.5f));
    tri_up_obj->SetTranslation(glm::vec3(0.0f, 0.6f, -1.0f));
    camera_obj->AddChild(std::shared_ptr<DObject>(tri_up_obj));

    return camera_obj;

  }

/* ================ Camera ================================*/
  static ModelObject* CreateModelObject(const std::string& path) {

    Model* model = new Model(path);

    // auto shader_model = std::make_shared<Shader>(
    //   "../shaders/one.vs",
    //   "../shaders/one_model.fs");

    auto shader_model = std::make_shared<Shader>(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ModelObject* model_obj =
        new ModelObject(model);
    model_obj->SetShader(shader_model);

    return model_obj;
  
  }

/* ================ Cube ================================*/
  static ColorObject* CreateCube(float size = 1.0) {

    auto mesh = MeshFactory::CreateCube();

    // auto shader_color = std::make_shared<Shader>(
    //   "../shaders/one.vs",
    //   "../shaders/one_color.fs");

    // auto shader_color = std::make_shared<Shader>(
    //     "../shaders/two.vs",
    //     "../shaders/two_model.fs");

    auto shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ColorObject* cube_obj =
        new ColorObject(mesh, glm::vec4(0.7f, 0.7f, 0.7f, 1.0f));
    cube_obj->SetShader(shader_color);

    return cube_obj;
  
  }

/* ================ Image ================================*/
  static ImageObject* CreateImage(const std::string image_path = "") {

    auto mesh = MeshFactory::CreateImagePlane();

    // auto shader_color = std::make_shared<Shader>(
    //   "../shaders/one.vs",
    //   "../shaders/one_color.fs");

    // auto shader_color = std::make_shared<Shader>(
    //     "../shaders/two.vs",
    //     "../shaders/two_model.fs");

    auto shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ImageObject* image_obj = new ImageObject(mesh);

    image_obj->SetShader(shader_color);
    
    if (!image_path.empty()) {
      image_obj->SetImage(image_path);
    }

    return image_obj;
  }

/* ================ Axes ================================*/
  static DObject* CreateAxes(float size = 1.0) {

    DObject* axes = new DObject();

    std::shared_ptr<ColorObject> zero_cube_obj(ObjectFactory::CreateCube());
    zero_cube_obj->SetScale(glm::vec3(0.5f * size));
    axes->AddChild(zero_cube_obj);

    std::shared_ptr<ColorObject> x_cube_obj(ObjectFactory::CreateCube());
    x_cube_obj->SetColor({0.8, 0.1, 0.1, 1.0});
    x_cube_obj->SetScale(glm::vec3(1.0f * size));
    x_cube_obj->SetTranslation(glm::vec3(10.0f * size, 0.0f, 0.0f));
    axes->AddChild(x_cube_obj);

    std::shared_ptr<ColorObject> y_cube_obj(ObjectFactory::CreateCube());
    y_cube_obj->SetColor({0.1, 0.8, 0.1, 1.0});
    y_cube_obj->SetScale(glm::vec3(1.0f * size));
    y_cube_obj->SetTranslation(glm::vec3(0.0f, 10.0f * size, 0.0f));
    axes->AddChild(y_cube_obj);

    std::shared_ptr<ColorObject> z_cube_obj(ObjectFactory::CreateCube());
    z_cube_obj->SetColor({0.1, 0.1, 0.8, 1.0});
    z_cube_obj->SetScale(glm::vec3(1.0f * size));
    z_cube_obj->SetTranslation(glm::vec3(0.0f, 0.0f, 10.0f * size));
    axes->AddChild(z_cube_obj);

    return axes;
  
  }

  /* ================ Points ================================*/
  static ColorObject* CreatePoints(std::vector<glm::vec3>& points, const glm::vec4& color = glm::vec4(1.0f, 0.4f, 0.4f, 1.0f)) {

    auto mesh = MeshFactory::CreatePoints(points);

    auto shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ColorObject* points_obj =
        new ColorObject(mesh, color);
    points_obj->SetShader(shader_color);
    points_obj->NoCorrection();


    return points_obj;

  }

  /* ================ Points Color ================================*/
  static ColorObject* CreatePoints(std::vector<Point3DColor>& points, 
          const bool extend_points = false,
          const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)) {

    auto mesh = MeshFactory::CreatePoints(points);

    std::shared_ptr<Shader> shader_color;
    if (extend_points) {
      shader_color = ObjectFactory::GetShader(
          "../shaders/points.vs",
          "../shaders/points.fs",
          "../shaders/points.gs");
    } else {
      shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");
    }

    ColorObject* points_obj =
        new ColorObject(mesh, color, true);
    points_obj->SetShader(shader_color);
    points_obj->NoCorrection();

    return points_obj;

  }


  /* ================ Camera ================================ */
  // static CameraObject* CreateCamera(const float width_ratio = 1.0) {
  //   return new CameraObject(width_ratio);
  // }

};

// Cache of shaders (reusable ones)
std::map<std::string, std::shared_ptr<Shader> > ObjectFactory::shaders_;

class CameraObject: public DObject {
  public:
  CameraObject(const CameraIntrinsics intr) : CameraObject(intr.wr, intr) {
    // std::cout << "CameraObject:con1 = ";
    // intr.print();
    // std::cout << std::endl;
  }

  CameraObject(const CameraIntrinsics intr, const int width, const int height)
      : CameraObject(intr.wr, {
          .fx = intr.fx / static_cast<float>(width),
          .fy = intr.fy / static_cast<float>(height),
          .cx = intr.cx / static_cast<float>(width),
          .cy = intr.cy / static_cast<float>(height)}) {}
/*
    std::cout << "CameraObject:con2 = ";
    intr.print();
    std::cout << std::endl;

    CameraIntrinsics cintr(intr);
    cintr.fx = cintr.fx / static_cast<double>(width);
    cintr.fy = cintr.fy / static_cast<double>(height);
    cintr.cx = cintr.cx / static_cast<double>(width);
    cintr.cy = cintr.cy / static_cast<double>(height);
    CameraObject(cintr.wr, cintr);
  }
*/

  CameraObject(const float width_ratio = 1.0f, const CameraIntrinsics intr = CameraIntrinsics())
      : DObject(nullptr, "CameraObject"), intrinsics_(intr), width_ratio_(width_ratio) {

    // std::cout << "CameraObject:con3 = ";
    // intr.print();
    // std::cout << std::endl;

    // std::cout << "camera: ";
    // intr.print();
    // std::cout << std::endl;

    // TODO: Move to the usage of intr.wr instead of separate width_ration var
    intrinsics_.wr = width_ratio;

    float f = std::max(intrinsics_.fx, intrinsics_.fy);

    // Construct complex object

    // ========== Create New Frustrum ===============
    std::vector<Texture> textures;
    std::vector<Vertex> vertices = {
      // {glm::vec3(-0.5f, -0.5f, f)},  // 0
      // {glm::vec3(- (1 - intrinsics_.cx) * width_ratio_, - intrinsics_.cy, f)},  // 0
      {glm::vec3(- intrinsics_.cx * width_ratio_, - intrinsics_.cy, f)},  // 0 TEST
      // {glm::vec3(0.5f, -0.5f, f)},   // 1
      // {glm::vec3(intrinsics_.cx * width_ratio_, - intrinsics_.cy, f)},   // 1
      {glm::vec3((1 - intrinsics_.cx) * width_ratio_, - intrinsics_.cy, f)},   // 1 TEST
      // {glm::vec3(0.5f, 0.5f, f)},  // 2
      // {glm::vec3(intrinsics_.cx * width_ratio_,  (1 - intrinsics_.cy), f)},  // 2
      {glm::vec3((1 - intrinsics_.cx) * width_ratio_,  (1 - intrinsics_.cy), f)},  // 2 TEST
      // {glm::vec3(-0.5f, 0.5f, f)}, // 3
      // {glm::vec3(- (1 - intrinsics_.cx) * width_ratio_, (1 - intrinsics_.cy), f)}, // 3
      {glm::vec3(- intrinsics_.cx * width_ratio_, (1 - intrinsics_.cy), f)}, // 3 TEST
      {glm::vec3(0.0f, 0.0f, 0.0f)}   // 4
    };
    std::vector<unsigned int> indices = {
      0, 1,
      1, 2,
      2, 3,
      3, 0,
      0, 4,
      1, 4,
      2, 4,
      3, 4
    };
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::LINES);
    auto shader_color = ObjectFactory::GetShader(
        "../shaders/two.vs",
        "../shaders/two_model.fs");
    std::shared_ptr<ColorObject> new_camera_obj = std::make_shared<ColorObject>(
        mesh, glm::vec4(1.0f, 0.7f, 0.7f, 1.0f));
    new_camera_obj->SetShader(shader_color);
    new_camera_obj->NoCorrection();
    this->AddChild(new_camera_obj);

    // Add Up Triangle
    auto tri_mesh = MeshFactory::CreateCameraUpTriangle();
    ColorObject* tri_up_obj =
        new ColorObject(tri_mesh, glm::vec4(0.1f, 0.1f, 0.1f, 1.0f));
    tri_up_obj->SetShader(shader_color);
    tri_up_obj->SetScale(glm::vec3(0.5f));
    // tri_up_obj->SetTranslation(glm::vec3(0.0f, 0.6f, f));
    // tri_up_obj->SetTranslation(glm::vec3((intrinsics_.cx * width_ratio_ - 0.5f * width_ratio_), (-intrinsics_.cy + 0.5f) + 0.55f, f));
    tri_up_obj->SetTranslation(glm::vec3((- intrinsics_.cx * width_ratio_ + 0.5f * width_ratio_), (-intrinsics_.cy + 0.5f) + 0.55f, f));  // TEST
    new_camera_obj->AddChild(std::shared_ptr<DObject>(tri_up_obj));

    camera_obj_ = new_camera_obj;


    /* - old camera object 
    // Camera Frustum
    camera_obj_ = std::shared_ptr<ColorObject>(ObjectFactory::CreateCameraFrustum());
    // glm::mat4 rr(1.0f);
    // rr = ;
    camera_obj_->SetRotation(glm::rotate(glm::mat4(1.0f), static_cast<float>(M_PI), glm::vec3(0.0f, 1.0f, 0.0f)));
    camera_obj_->SetScale(glm::vec3(width_ratio, 1.0f, f));
    // camera_obj_->SetTranslation(glm::vec3(0.0f, 0.0f, 0.0f));
    // this->AddChild(camera_obj_);
    */

    // Image Plane
    image_obj_ = std::shared_ptr<ImageObject>(ObjectFactory::CreateImage());
    image_obj_->SetScale(glm::vec3(width_ratio, 1.0f, 1.0f));
    // image_obj_->SetTranslation(glm::vec3((intrinsics_.cx * width_ratio_ - 0.5f * width_ratio_), (-intrinsics_.cy + 0.5f), f));
    image_obj_->SetTranslation(glm::vec3((- intrinsics_.cx * width_ratio_ + 0.5f * width_ratio_), (-intrinsics_.cy + 0.5f), f));
    this->AddChild(image_obj_);

  // fs::path image_path = camera1_image_path / fs::path(camera1_poses[0].filename);
  // image_obj->SetImage(image_path.string(), false);
  // full_camera_obj->AddChild(image_obj);

  }

  void SetImage(const std::string& image_path) {
    if (image_obj_) {
      image_obj_->SetImage(image_path, false);
    }
  }

  void SetImage(const cv::Mat& image) {
    if (image_obj_) {
      image_obj_->SetImage(image, false);
    }
  }

  void SetImageAlpha(const float alpha) {
    if (image_obj_) {
      image_obj_->SetImageAlpha(alpha);
    }
  }

  void SetImageTransparency(const bool transparency) {
    if (image_obj_) {
      image_obj_->SetImageTransparency(transparency);
    }
  }

  CameraIntrinsics GetCameraIntrinsics() const {
    return intrinsics_;
  }

  void AddProjectedPoints(const std::vector<glm::vec3>& points) {
    // projected_points_.insert(projected_points_.end(), points.begin(), points.end());

    float f = std::max(intrinsics_.fx, intrinsics_.fy);

    // std::cout << "intr_.cx = " << intrinsics_.cx << std::endl;
    // std::cout << "intr_.cy = " << intrinsics_.cy << std::endl;

    // Translate to Camera Coordinate System
    
    std::vector<glm::vec3> p(points);
    // std::cout << "proj_points.size = " << p.size() << std::endl;
    for (int i = 0; i < p.size(); ++i) {
      // std::cout << "p[" << i << "] = " << glm::to_string(p[i]) << std::endl;
      p[i] -= translation_;
      glm::vec3 r = glm::vec3(glm::transpose(rotation_) * glm::vec4(p[i], 1.0f));
      float sx = intrinsics_.fx * r[0] / r[2]; // + intrinsics_.cx; // TODO: Add s to calculation
      float sy = intrinsics_.fy * r[1] / r[2]; // + intrinsics_.cy;

      float ssx = sx * width_ratio_; // - intrinsics_.cx * width_ratio_;
      float ssy = sy; // - intrinsics_.cy;
      float ssz = f;
      p[i] = glm::vec3(ssx, ssy, ssz);
      // std::cout << "p[" << i << "] = " << glm::to_string(p[i]) << std::endl;
      // std::cout << "r[" << i << "] = " << glm::to_string(r) << std::endl;
      // std::cout << "sx, sy = " << sx << ", " << sy << std::endl;
      // std::cout << "ssx, ssy, ssz = " << ssx << ", " << ssy << ", " << ssz << std::endl;
    }

    // Corner Points
    // std::vector<glm::vec3> cp(1);
    // cp[0] = glm::vec3((1 - intrinsics_.cx) * width_ratio_, -intrinsics_.cy, f);
    // cp[0] = glm::vec3(-intrinsics_.cx * width_ratio_, -intrinsics_.cy, f);
    // cp[1] = glm::vec3(-(1 - intrinsics_.cx) * width_ratio_, -intrinsics_.cy, f);
    // cp[2] = glm::vec3(-(1 - intrinsics_.cx) * width_ratio_, (1 - intrinsics_.cy), f);
    // cp[3] = glm::vec3(intrinsics_.cx * width_ratio_, (1 - intrinsics_.cy), f);
    // std::shared_ptr<DObject> cp_points_obj(ObjectFactory::CreatePoints(cp));
    // this->AddChild(cp_points_obj);

    std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(p));
    this->AddChild(points_obj);

  }

void AddProjectedPoints(const std::vector<Point3DColor>& points) {

    float f = std::max(intrinsics_.fx, intrinsics_.fy);

    // Translate to Camera Coordinate System
    std::vector<glm::vec3> p_glm(points.size());
    std::vector<Point3DColor> p(points);
    // std::cout << "proj_points.size = " << p.size() << std::endl;
    for (int i = 0; i < p.size(); ++i) {
      // std::cout << "p[" << i << "] = " << glm::to_string(p[i]) << std::endl;
      // p[i] = points[i].pt;
      p[i].pt -= translation_;
      glm::vec3 r = glm::vec3(glm::transpose(rotation_) * glm::vec4(p[i].pt, 1.0f));
      float sx = intrinsics_.fx * r[0] / r[2]; // + intrinsics_.cx; // TODO: Add s to calculation
      float sy = intrinsics_.fy * r[1] / r[2]; // + intrinsics_.cy;

      float ssx = sx * width_ratio_; // - intrinsics_.cx * width_ratio_;
      float ssy = sy; // - intrinsics_.cy;
      float ssz = f;
      p[i].pt = glm::vec3(ssx, ssy, ssz);
      p_glm[i] = p[i].pt;
      // std::cout << "p[" << i << "] = " << glm::to_string(p[i]) << std::endl;
      // std::cout << "r[" << i << "] = " << glm::to_string(r) << std::endl;
      // std::cout << "sx, sy = " << sx << ", " << sy << std::endl;
      // std::cout << "ssx, ssy, ssz = " << ssx << ", " << ssy << ", " << ssz << std::endl;
    }

    // std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(p));
    std::shared_ptr<DObject> points_obj(ObjectFactory::CreatePoints(p_glm));
    this->AddChild(points_obj);

  }

private:
  std::shared_ptr<ImageObject> image_obj_;
  std::shared_ptr<ColorObject> camera_obj_;
  CameraIntrinsics intrinsics_;
  float width_ratio_;
  // std::vector<glm::vec3> projected_points_;
};


#endif  // CV_GL_OBJECT_FACTORY_H_
