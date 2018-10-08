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


const int kWindowWidth = 1226/2;
const int kWindowHeight = 1028/2;

// float last_x = kWindowWidth / 2;
// float last_y = kWindowHeight / 2;

// bool first_mouse;

// float delta_time, last_time = 0;

std::shared_ptr<Mesh> MakeFloor(const double step_size,
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

std::shared_ptr<Mesh> MakeCameraCone() {
  // std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;

  std::vector<Vertex> vertices = {
    {glm::vec3(-0.5f, 0.0f, 0.0f)},
    {glm::vec3(0.5f, 0.0f, 0.0f)},
    {glm::vec3(0.0f, 0.5f, 0.0f)}
  };

  auto mesh = std::make_shared<Mesh>(vertices, indices, textures);
  mesh->SetMeshType(MeshType::TRIANGLES);
  return mesh;
}


int main(int argc, char* argv[]) {

  std::shared_ptr<Camera> camera =
      std::make_shared<Camera>(glm::vec3(-3.0f, 0.0f, 1.5f));

  std::cout << "Hello edit space" << std::endl;

  glm::vec4 v(1.0f);
  std::cout << "v = " << v << std::endl;

  GLWindow gl_window("OpenGL: Edit Space", kWindowWidth, kWindowHeight);
  gl_window.SetCamera(camera);

  auto shader_color = std::make_shared<Shader>(
    "../shaders/one.vs",
    "../shaders/one_color.fs");

  auto mesh_floor = MakeFloor(1.0, 32);
  std::cout << "mesh_floor (init) = " << mesh_floor << std::endl;

  auto mesh_camera = MakeCameraCone();
  std::cout << "mesh_camera (init) = " << mesh_camera << std::endl;


  // Renderer
  std::unique_ptr<Renderer> renderer(new Renderer(camera));

  // Create DObject
  ColorObject floor_obj(mesh_floor, glm::vec4(0.7f, 0.7f, 1.0f, 1.0f));
  floor_obj.SetShader(shader_color);
  floor_obj.SetTranslation(glm::vec3(0.0f, 0.0f, 0.0f));
  // floor_obj.SetScale(glm::vec3(0.2f));

  ColorObject camera_obj(mesh_camera, glm::vec4(1.0f, 0.7f, 0.7f, 1.0f));
  camera_obj.SetShader(shader_color);
  // camera_obj.SetTranslation(glm::vec3(0.0f, 0.0f, 5.0f));
  // floor_obj.SetScale(glm::vec3(0.2f));

  std::cout << "Floor = " << floor_obj << std::endl;
  std::cout << "Camer = " << camera_obj << std::endl;

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    /* ====================== Rend ===================== */
    renderer->Draw(floor_obj);
    renderer->Draw(camera_obj);


    gl_window.RunLoop();
  }

  return EXIT_SUCCESS;

}
