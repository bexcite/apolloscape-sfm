// Copyright Pavlo 2017

// Links

// OpenCV Camera to OpenGL Projection
// https://blog.noctua-software.com/opencv-opengl-projection-matrix.html

// calculating OpenGL perspective matrix from OpenCV intrinsic matrix
// http://kgeorge.github.io/2014/03/08/calculating-opengl-perspective-matrix-from-opencv-intrinsic-matrix

// OpenGL 3.2 API Quick Reference Card
// http://www.glprogramming.com/manpages/opengl-quick-reference-card.pdf

// Calibrated Cameras in OpenGL without glFrustum
// http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/

// A trip through the Graphics Pipeline 2011: Index [IN-DEPTH]
// https://fgiesen.wordpress.com/2011/07/09/a-trip-through-the-graphics-pipeline-2011-index/

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// #define STB_IMAGE_IMPLEMENTATION
// #include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <iostream>

#include "cv_gl/shader.h"
#include "cv_gl/camera.h"
#include "cv_gl/mesh.h"
#include "cv_gl/model.h"


// Define Some Constants
const int kWindowWidth = 1280;
const int kWindowHeight = 800;

float last_x = kWindowWidth / 2;
float last_y = kWindowHeight / 2;

bool first_mouse;

float delta_time, last_time = 0;

void processInput(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);




// #define TEST_ENABLE

void test_mesh() {
  // Shader shader(
  //   "../shaders/one.vs",
  //   "../shaders/one.fs");
  // shader.PrintHello();
  std::cout << "hello from mesh testing" << std::endl;
  Vertex v = {.position = {0.0f, 0.0f, 0.0f}};
  std::vector<Vertex> vertices = {
    {{0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
    {{0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
    {{-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
    {{-0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}},
  };
  std::vector<unsigned int> indices = {0, 1, 3, 1, 2, 3};
  std::vector<Texture> textures;
  Mesh mesh(vertices, indices, textures);
  std::cout << "print = " << glm::to_string(vertices[0].position) << std::endl;
  std::cout << "print 2 = " << *(((float *)&vertices[0]) + 15) << std::endl;
}


Camera camera(glm::vec3(2.0f, 0.0f, 3.0f));


Mesh MakeRect() {
  std::vector<Vertex> vertices_mesh = {
    {{0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},  // >^
    {{0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},  // >.
    {{-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}}, // <.
    {{-0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}}, // <^
  };
  std::vector<unsigned int> indices_mesh = {0, 1, 3, 1, 2, 3};
  std::vector<Texture> textures_mesh;
  return Mesh(vertices_mesh, indices_mesh, textures_mesh);
}

Mesh MakeTriangle() {
  std::vector<Vertex> vertices_mesh = {
    {{0.5f,  1.5f, -3.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
    {{0.5f, -1.5f, -3.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
    {{-0.5f, -1.5f, -3.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}}
  };
  std::vector<unsigned int> indices_mesh = {0, 1, 2};
  std::vector<Texture> textures_mesh;
  return Mesh(vertices_mesh, indices_mesh, textures_mesh);
}




int main(int argc, char* argv[]) {

#ifdef TEST_ENABLE
  test_mesh();
  // Rendering Loop
  /*
  while (glfwWindowShouldClose(window) == false) {
    processInput(window);
    // Flip Buffers and Draw
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  glfwTerminate();
  */
  return 0;
#endif


  // Load GLFW and Create a Window
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight, "OpenGL",
      nullptr, nullptr);

  // Check for Valid Context
  if (window == nullptr) {
      fprintf(stderr, "Failed to Create OpenGL Context");
      glfwTerminate();
      return EXIT_FAILURE;
  }


  // Create Context and Load OpenGL Functions
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetScrollCallback(window, scroll_callback);

  gladLoadGL();
  fprintf(stderr, "OpenGL %s, %s, %s\n", glGetString(GL_VERSION),
      glGetString(GL_VENDOR), glGetString(GL_RENDERER));




  //  Create shader program
  Shader shader(
    "../shaders/one.vs",
    "../shaders/one.fs");


  Vertex v = {{0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}};
  std::cout << "Test V: " << v << std::endl;


  // std::vector<Vertex> vertices_mesh = {
  //   {{0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
  //   {{0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
  //   {{-0.5f, -0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
  //   {{-0.5f,  0.5f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}},
  // };
  // std::vector<unsigned int> indices_mesh = {0, 1, 3, 1, 2, 3};
  // std::vector<Texture> textures_mesh;
  // Mesh mesh(vertices_mesh, indices_mesh, textures_mesh);



  // setup vertex data
  float vertices[] = {
    // coords 3         // color 3         // tex coords 2
     0.5f,  0.5f, 0.0f,  1.0f, 0.0f, 0.0f,  1.0f, 1.0f,  // top right 0
     0.5f, -0.5f, 0.0f,  0.0f, 1.0f, 0.0f,  1.0f, 0.0f,  // bottom right 1
    -0.5f, -0.5f, 0.0f,  0.0f, 0.0f, 1.0f,  0.0f, 0.0f,  // bottom left 2
    -0.5f,  0.5f, 0.0f,  1.0f, 0.0f, 0.0f,  0.4f, 0.6f   // top left 3
    // 0.0f, 0.75f, 0.0f, 0.0f, 1.0f, 0.0f,   // center top 4
    // 0.0f, -0.75f, 0.0f, 0.0f, 0.0f, 1.0f   // center bottom 5
  };

  unsigned int indices[] = {
    0, 1, 3,
    1, 2, 3
  };

  Mesh mesh_rect = MakeRect();
  Mesh mesh_tri = MakeTriangle();
  std::cout << "mesh_rect (init) = " << mesh_rect << std::endl;
  std::cout << "mesh_tri (init) = " << mesh_tri << std::endl;

  glm::mat4 m_model(1.0f);
  m_model = glm::scale(m_model, glm::vec3(3.0f, 3.0f, 3.0f));


  Model m("../data/objects/nanosuit/nanosuit.obj");
  
  // m.meshes_.emplace_back(mesh_tri);
  // m.meshes_.emplace_back(mesh_rect);

  std::cout << "mesh_rect (late) = " << mesh_rect << std::endl;
  std::cout << "mesh_tri (late) = " << mesh_tri << std::endl;



  // std::cout << "MODEL MESHES: " << m.meshes_.size() << std::endl;
  //
  // Mesh msh = m.meshes_[0];
  // for (unsigned int i = 0; i < msh.vertices.size(); ++i) {
  //   std::cout << "mv: "
  //       << glm::to_string(m_model * glm::vec4(msh.vertices[i].position, 1.0f))
  //       << std::endl;
  //   if (i < msh.indices.size()) {
  //     std::cout << "mi: " << msh.indices[i] << std::endl;
  //   }
  // }



  // camera.SetPosition(glm::vec3(-2.785094f, 39.048775f, -4.100138f));
  // camera.SetDirection(msh.vertices[0].position);

  // camera.SetDirection(glm::vec3(2.0f, 0.0f, -1.0f));

/*
  unsigned int VBO, VAO, EBO;

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
      GL_STATIC_DRAW);

  // position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
      (void*)0);
  glEnableVertexAttribArray(0);

  //color attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
      (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  //tex coords
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float),
      (void*)(6 * sizeof(float)));
  glEnableVertexAttribArray(2);

  // Ubind buffer
  glBindBuffer(GL_ARRAY_BUFFER, 0);
*/


  // Load and bind texture
  unsigned int texture1, texture2;
  unsigned char *tex_data;
  int width, height, nr_channels;

  // we need to flip all images for texture processing
  stbi_set_flip_vertically_on_load(true);

  // texture 1 setup
  glGenTextures(1, &texture1);
  glBindTexture(GL_TEXTURE_2D, texture1);

  // set the texture wrapping/filtering options
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // load and generate the texture
  tex_data = stbi_load("../data/170427_222949577_Camera_1.jpg",
      &width, &height, &nr_channels, 0);
  printf("Image 1: %d, %d, %d\n", width, height, nr_channels);
  if (tex_data) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
        GL_UNSIGNED_BYTE, tex_data);
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    std::cout << "Error loading texture 1" << std::endl;
  }
  stbi_image_free(tex_data);

  // texture 2 setup
  glGenTextures(1, &texture2);
  glBindTexture(GL_TEXTURE_2D, texture2);

  // set the texture wrapping/filtering options
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  tex_data = stbi_load("../data/170427_222949577_Camera_2.jpg",
      &width, &height, &nr_channels, 0);
  printf("Image 2: %d, %d, %d\n", width, height, nr_channels);
  if (tex_data) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
        GL_UNSIGNED_BYTE, tex_data);
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    std::cout << "Error loading texture 2" << std::endl;
  }
  stbi_image_free(tex_data);

  std::cout << "Textures: " << texture1 << ", " << texture2 << std::endl;


  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glEnable(GL_DEPTH_TEST);

  float timeSince = 0;
  unsigned int frame = 0;


  shader.Use();

  // set texture locations
  // shader.SetInt("texture1", 0);
  // shader.SetInt("texture2", 1);



  // unsigned int n_pos = 10;
  // glm::vec3 positions[] = {
  //   glm::vec3( 0.0f,  0.0f,  0.0f),
  //   glm::vec3( 2.0f,  5.0f, -15.0f),
  //   glm::vec3(-1.5f, -2.2f, -2.5f),
  //   glm::vec3(-3.8f, -2.0f, -12.3f),
  //   glm::vec3( 2.4f, -0.4f, -3.5f),
  //   glm::vec3(-1.7f,  3.0f, -7.5f),
  //   glm::vec3( 1.3f, -2.0f, -2.5f),
  //   glm::vec3( 1.5f,  2.0f, -2.5f),
  //   glm::vec3( 1.5f,  0.2f, -1.5f),
  //   glm::vec3(-1.3f,  1.0f, -1.5f)
  // };







  // Rendering Loop
  while (glfwWindowShouldClose(window) == false) {
      processInput(window);

      // Background Fill Color
      glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      float timeValue = glfwGetTime();
      delta_time = timeValue - last_time;
      last_time = timeValue;
      // std::cout << "delta_time = " << delta_time << std::endl;
      // std::cout << "getTime = " << timeValue << std::endl;

      float fps = 1.0f / (timeValue - timeSince);
      timeSince = timeValue;
      if (frame % 100 == 0) {
        std::cout << "FPS = " << fps << std::endl;
      }
      ++frame;

      float greenValue = (sin(timeValue) / 2.0f + 0.5f);

      shader.Use();
      shader.SetVector4f("ourColor", greenValue, greenValue, greenValue, 1.0f);

      // glm::mat4 transform;
      // transform = glm::rotate(trans, static_cast<float>(timeValue),
      //     glm::vec3(0.0, 0.0, 1.0));

      glm::mat4 model(1.0f);
      // float rotation_angle = timeValue * glm::radians(50.0f);
      // model = glm::rotate(model, rotation_angle,
      //     glm::vec3(0.5f, 1.0f, 0.0f));
      // shader.SetMatrix4fv("model", glm::value_ptr(model));

      // view = glm::lookAt(camera_pos, camera_pos + camera_front, up);

      glm::mat4 model_obj(1.0f);
      model_obj = glm::scale(model_obj, glm::vec3(3.0f, 3.0f, 3.0f));

      glm::mat4 view = camera.GetViewMatrix();

      glm::mat4 projection = glm::perspective(glm::radians(camera.GetZoom()),
          (float) kWindowWidth / kWindowHeight, 0.1f, 100.0f);


      shader.SetMatrix4fv("view", glm::value_ptr(view));
      shader.SetMatrix4fv("projection", glm::value_ptr(projection));
      shader.SetMatrix4fv("model", glm::value_ptr(m_model));

      // bind textures
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture1);
      shader.SetInt("texture1", 0);

      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, texture2);
      shader.SetInt("texture2", 1);

      // mesh.Draw(shader);

      // m.Draw(shader);

      mesh_rect.Draw(shader);
      mesh_tri.Draw(shader);

      /*
      // bind VAO data with vertex buffer, attributes and elements
      glBindVertexArray(VAO);

      for (unsigned int i = 0; i < n_pos; ++i) {
        glm::mat4 model_obj(1.0f);
        model_obj = glm::translate(model, positions[i]);
        // float rot_angle = 20.0f * i + rotation_angle;
        // model_obj = glm::rotate(model_obj, rot_angle,
        //       glm::vec3(1.0f, 0.3f, 0.5f));
        shader.SetMatrix4fv("model", glm::value_ptr(model_obj));
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        // glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, 0);

      }
      */

      // glDrawArrays(GL_TRIANGLES, 0, 6);
      // glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

      // Flip Buffers and Draw
      glfwSwapBuffers(window);
      glfwPollEvents();
  }

  // clear gl objects
  // glDeleteVertexArrays(1, &VAO);
  // glDeleteBuffers(1, &VBO);
  // glDeleteBuffers(1, &EBO);


  glfwTerminate();

  return EXIT_SUCCESS;
}


void processInput(GLFWwindow *window) {

  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
  } else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    camera.ProcessKeyboard(FORWARD, delta_time);
  } else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    camera.ProcessKeyboard(BACKWARD, delta_time);
  } else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    camera.ProcessKeyboard(LEFT, delta_time);
  } else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    camera.ProcessKeyboard(RIGHT, delta_time);
  }
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos) {
  if (first_mouse) {
    last_x = xpos;
    last_y = ypos;
    first_mouse = false;
  }

  float xoffset = xpos - last_x;
  float yoffset = last_y - ypos;

  last_x = xpos;
  last_y = ypos;

  camera.ProcessMouseInput(xoffset, yoffset);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  camera.ProcessMouseScroll(yoffset);
}
