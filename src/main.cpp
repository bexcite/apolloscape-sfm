// Copyright Pavlo 2017

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <iostream>

#include "cv_gl/shader.h"


// Define Some Constants
const int kWindowWidth = 1280;
const int kWindowHeight = 800;

void processInput(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);


// #define TEST_ENABLE
/*
void test_shader() {
  Shader shader(
    "../shaders/one.vs",
    "../shaders/one.fs");
  shader.PrintHello();
}
*/



int main(int argc, char* argv[]) {
  // std::cout << "Hello world!" << std::endl;



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

  // GLFWglproc gls = glfwGetProcAddress("glGetString");

  // Create Context and Load OpenGL Functions
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


  // if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
  //   fprintf(stderr, "Failed to initialize GLAD\n");
  //   // return EXIT_FAILURE;
  // }

  gladLoadGL();
  fprintf(stderr, "OpenGL %s, %s, %s\n", glGetString(GL_VERSION),
      glGetString(GL_VENDOR), glGetString(GL_RENDERER));

/*
#ifdef TEST_ENABLE
  test_shader();
  // Rendering Loop
  while (glfwWindowShouldClose(window) == false) {
    processInput(window);
    // Flip Buffers and Draw
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  glfwTerminate();
  return 0;
#endif
*/

  //  Create shader program
  Shader shader(
    "../shaders/one.vs",
    "../shaders/one.fs");

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

  // unsigned int indices[] = {
  //   0, 1, 3,
  //   1, 2, 3,
  //   0, 3, 4,
  //   1, 2, 5
  // };

  unsigned int indices[] = {
    0, 1, 3,
    1, 2, 3
    // 0, 3, 4,
    // 1, 2, 5
  };

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

  float timeSince = 0;
  unsigned int frame = 0;


  // set shaders locations
  shader.Use();
  shader.SetInt("texture1", 0);
  shader.SetInt("texture2", 1);

  // Rendering Loop
  while (glfwWindowShouldClose(window) == false) {
      processInput(window);

      // Background Fill Color
      glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      float timeValue = glfwGetTime();
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

      // bind textures
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture1);
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, texture2);

      // bind VAO data with vertex buffer, attributes and elements
      glBindVertexArray(VAO);

      // glDrawArrays(GL_TRIANGLES, 0, 6);
      glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

      // Flip Buffers and Draw
      glfwSwapBuffers(window);
      glfwPollEvents();
  }

  // clear gl objects
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &EBO);


  glfwTerminate();

  return EXIT_SUCCESS;
}


void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}
