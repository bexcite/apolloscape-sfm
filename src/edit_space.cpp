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

float last_x = kWindowWidth / 2;
float last_y = kWindowHeight / 2;

bool first_mouse;

float delta_time, last_time = 0;

// void processInput(GLFWwindow *window);
// void framebuffer_size_callback(GLFWwindow *window, int width, int height);
// void mouse_callback(GLFWwindow *window, double xpos, double ypos);
// void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);


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
  DObject floor_obj(mesh_floor);
  floor_obj.SetShader(shader_color);
  floor_obj.SetTranslation(glm::vec3(0.0f, 0.0f, 10.0f));

  std::cout << "Floor = " << floor_obj << std::endl;

  while(gl_window.IsRunning()) {
    // std::cout << "delta_time = " << gl_window.delta_time << std::endl;

    // rotate to world coords
    glm::mat4 model_matrix;
    model_matrix[0] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
    model_matrix[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
    model_matrix[2] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    model_matrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    model_matrix = glm::transpose(model_matrix);
    // model_matrix = glm::scale(model_matrix, glm::vec3(1.0f, 1.0f, 1.0f));

    glm::mat4 view_matrix = camera->GetViewMatrix();

    glm::mat4 projection_matrix = camera->GetProjMatrix();

    /* ============= FLOOR ====================== */
    shader_color->Use();

    shader_color->SetMatrix4fv("view", glm::value_ptr(view_matrix));
    shader_color->SetMatrix4fv("projection", glm::value_ptr(projection_matrix));
    shader_color->SetMatrix4fv("model", glm::value_ptr(model_matrix));

    glm::vec4 floor_color = glm::vec4(0.7f, 0.7f, 1.0f, 1.0f);

    shader_color->SetVector4fv("color", glm::value_ptr(floor_color));
    mesh_floor->Draw(shader_color);

    /* ============= CAMERA ====================== */
    shader_color->Use();

    shader_color->SetMatrix4fv("view", glm::value_ptr(view_matrix));
    shader_color->SetMatrix4fv("projection", glm::value_ptr(projection_matrix));
    shader_color->SetMatrix4fv("model", glm::value_ptr(model_matrix));

    glm::vec4 camera_color = glm::vec4(1.0f, 0.7f, 0.7f, 1.0f);

    shader_color->SetVector4fv("color", glm::value_ptr(camera_color));
    mesh_camera->Draw(shader_color);

    /* ====================== Rend ===================== */
    renderer->Draw(floor_obj);


    gl_window.RunLoop();
  }

  return EXIT_SUCCESS;

}
