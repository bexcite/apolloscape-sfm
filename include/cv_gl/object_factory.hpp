// Copyright Pavlo 2018
#ifndef CV_GL_OBJECT_FACTORY_H_
#define CV_GL_OBJECT_FACTORY_H_

#include <iostream>

#include "cv_gl/mesh.h"
#include "cv_gl/shader.h"

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


};


class ObjectFactory {

public:

/* ================ Floor ================================*/
  static ColorObject* CreateFloor(const double step_size,
      const unsigned int line_num) {

    auto mesh = MeshFactory::CreateGrid(step_size, line_num);

    // auto shader_color = std::make_shared<Shader>(
    //   "../shaders/one.vs",
    //   "../shaders/one_color.fs");

    auto shader_color = std::make_shared<Shader>(
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

    auto shader_color = std::make_shared<Shader>(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ColorObject* camera_obj =
        new ColorObject(mesh, glm::vec4(1.0f, 0.7f, 0.7f, 1.0f));
    camera_obj->SetShader(shader_color);

    // Add additional correction to look forward
    glm::mat4 correction(1.0f);
    // correction = glm::rotate(correction, static_cast<float>(-M_PI_2), glm::vec3(0.0f, 1.0f, 0.0f));
    correction = glm::translate(correction, glm::vec3(0.0f, 0.0f, -2.0f));
    camera_obj->AddToCorrectionMatrix(correction);

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

    auto shader_color = std::make_shared<Shader>(
        "../shaders/two.vs",
        "../shaders/two_model.fs");

    ColorObject* cube_obj =
        new ColorObject(mesh, glm::vec4(0.7f, 0.7f, 0.7f, 1.0f));
    cube_obj->SetShader(shader_color);

    return cube_obj;
  
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




};


#endif  // CV_GL_OBJECT_FACTORY_H_
