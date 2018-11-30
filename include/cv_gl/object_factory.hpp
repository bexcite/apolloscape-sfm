// Copyright Pavlo 2018
#ifndef CV_GL_OBJECT_FACTORY_H_
#define CV_GL_OBJECT_FACTORY_H_

#include <iostream>

#include "cv_gl/mesh.h"
#include "cv_gl/shader.h"


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
      {glm::vec3(-1.0f, 0.0f, 1.0f)},  // 0
      {glm::vec3(1.0f, 0.0f, 1.0f)},   // 1
      {glm::vec3(1.0f, 0.0f, -1.0f)},  // 2
      {glm::vec3(-1.0f, 0.0f, -1.0f)}, // 3
      {glm::vec3(0.0f, 2.0f, 0.0f)},   // 4
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
    std::vector<unsigned int> indices = {
      0, 1, 2, 
      0, 2, 3,
      4, 5, 6,
      4, 6, 7,
      0, 3, 4,
      4, 3, 7,
      0, 1, 5,
      0, 5, 4,
      3, 2, 6,
      3, 6, 7,
      1, 2, 6,
      1, 6, 5
    };
    auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
    mesh->SetMeshType(MeshType::TRIANGLES);
    return mesh;
  }


};


class ObjectFactory {

public:

/* ================ Floor ================================*/
  static ColorObject* CreateFloor(const double step_size,
      const unsigned int line_num) {

    auto mesh = MeshFactory::CreateGrid(step_size, line_num);

    auto shader_color = std::make_shared<Shader>(
      "../shaders/one.vs",
      "../shaders/one_color.fs");

    ColorObject* floor_obj =
        new ColorObject(mesh, glm::vec4(0.7f, 0.7f, 1.0f, 1.0f));
    floor_obj->SetShader(shader_color);

    return floor_obj;

  }


/* ================ Camera ================================*/
  static ColorObject* CreateCameraFrustum() {

    auto mesh = MeshFactory::CreateCameraFrustum();

    auto shader_color = std::make_shared<Shader>(
      "../shaders/one.vs",
      "../shaders/one_color.fs");

    ColorObject* camera_obj =
        new ColorObject(mesh, glm::vec4(1.0f, 0.7f, 0.7f, 1.0f));
    camera_obj->SetShader(shader_color);

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

    auto shader_color = std::make_shared<Shader>(
      "../shaders/one.vs",
      "../shaders/one_color.fs");

    ColorObject* cube_obj =
        new ColorObject(mesh, glm::vec4(1.0f, 0.7f, 0.7f, 1.0f));
    cube_obj->SetShader(shader_color);

    return cube_obj;
  
  }




};


#endif  // CV_GL_OBJECT_FACTORY_H_
