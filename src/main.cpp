// Copyright Pavlo 2017

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

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


// Define Some Constants
const int kWindowWidth = 1280;
const int kWindowHeight = 800;

float last_x = kWindowWidth / 2;
float last_y = kWindowHeight / 2;

bool first_mouse;
float yaw = 90.0f, pitch = 0;
glm::vec3 camera_front = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 camera_pos = glm::vec3(2.0f, 0.0f, 3.0f);

glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

float delta_time, last_time = 0;

void processInput(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);


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

  // test glm
  // glm::vec4 vec(1.0f, 0.0f, 0.0f, 1.0f);
  glm::mat4 trans(1.0f);
  // trans = glm::translate(trans, glm::vec3(1.0f, 1.0f, 0.0f));
  // trans = glm::rotate(trans, glm::radians(90.0f), glm::vec3(0.0, 0.0, 1.0));
  trans = glm::scale(trans, glm::vec3(0.5, 0.5, 0.5));
  // trans = glm::translate(trans, glm::vec3(0.5f, 0.5f, 0.0f));

  // vec = trans * vec;
  std::cout << "trans: \n" << glm::to_string(trans) << std::endl;
  // std::cout << "vec: \n" << glm::to_string(vec) << std::endl;



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
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
  glfwSetCursorPosCallback(window, mouse_callback);



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
  glEnable(GL_DEPTH_TEST);

  float timeSince = 0;
  unsigned int frame = 0;


  shader.Use();

  // set shaders locations
  shader.SetInt("texture1", 0);
  shader.SetInt("texture2", 1);

  // setup transforms


  // glm::mat4 view(1.0f);
  // view = glm::translate(view, glm::vec3(0.0f, 0.0f, -4.0f));

  glm::mat4 projection(1.0f);
  projection = glm::perspective(glm::radians(45.0f),
      (float) kWindowWidth / kWindowHeight, 0.1f, 100.0f);


  unsigned int n_pos = 10;
  glm::vec3 positions[] = {
    glm::vec3( 0.0f,  0.0f,  0.0f),
    glm::vec3( 2.0f,  5.0f, -15.0f),
    glm::vec3(-1.5f, -2.2f, -2.5f),
    glm::vec3(-3.8f, -2.0f, -12.3f),
    glm::vec3( 2.4f, -0.4f, -3.5f),
    glm::vec3(-1.7f,  3.0f, -7.5f),
    glm::vec3( 1.3f, -2.0f, -2.5f),
    glm::vec3( 1.5f,  2.0f, -2.5f),
    glm::vec3( 1.5f,  0.2f, -1.5f),
    glm::vec3(-1.3f,  1.0f, -1.5f)
  };


  // Camera matrix
  // glm::vec3 camera_pos = glm::vec3(2.0f, 0.0f, 3.0f);
  glm::vec3 camera_target = glm::vec3(2.0f, 0.0f, 0.0f);
  glm::vec3 camera_direction = glm::normalize(camera_pos - camera_target);


  glm::vec3 camera_right = glm::normalize(glm::cross(up, camera_direction));

  glm::vec3 camera_up = glm::normalize(
      glm::cross(camera_direction, camera_right));

  glm::mat4 look_at;
  look_at[0] = glm::vec4(camera_right, camera_pos[0]);
  look_at[1] = glm::vec4(camera_up, camera_pos[1]);
  look_at[2] = glm::vec4(camera_direction, camera_pos[2]);
  look_at[3] = glm::vec4(camera_pos, 1.0f);

  // glm::mat4 tr(1.0f);
  // tr = glm::translate(tr, camera_pos);
  // glm::mat4 look_at_mult = look_at * tr;
  // look_at = glm::translate(look_at, camera_pos);

  // std::cout << "look_at mult = " << glm::to_string(look_at_mult) << std::endl;


  // std::cout << "camera_direction (z) = " << glm::to_string(camera_direction)
  //     << std::endl;
  // std::cout << "camera_right (x) = " << glm::to_string(camera_right)
  //     << std::endl;
  // std::cout << "camera_up (y) = " << glm::to_string(camera_up)
  //     << std::endl;

  // std::cout << "look_at = " << glm::to_string(look_at) << std::endl;

  glm::mat4 view(1.0f);
  view = glm::lookAt(camera_pos, camera_pos + camera_front, up);

  std::cout << "================================" << std::endl;
  std::cout << "look_at tr  = " << glm::to_string(look_at) << std::endl;

  std::cout << "================================" << std::endl;
  std::cout << "look_at inv = " << glm::to_string(glm::affineInverse(look_at))
      << std::endl;

  std::cout << "================================" << std::endl;
  std::cout << "view1       = " << glm::to_string(view)
      << std::endl;

  glm::vec3 target_test(1.0f, 0.0f, 0.0f);
  glm::vec4 target1 = glm::affineInverse(look_at) * glm::vec4(target_test, 1.0f);
  glm::vec4 target2 = view * glm::vec4(target_test, 1.0f);
  std::cout << "-------------------------------" << std::endl;
  std::cout << "target1 = " << glm::to_string(target1)
      << std::endl;
  std::cout << "-------------------------------" << std::endl;
  std::cout << "target2 = " << glm::to_string(target2)
      << std::endl;

  std::cout << "-------------------------------" << std::endl;
  glm::vec4 target_proj = projection * target1;
  target_proj = target_proj / target_proj[3];
  std::cout << "target proj = " << glm::to_string(target_proj)
      << std::endl;



  // shader.SetMatrix4fv("transform", glm::value_ptr(trans));

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
      float rotation_angle = timeValue * glm::radians(50.0f);
      // model = glm::rotate(model, rotation_angle,
      //     glm::vec3(0.5f, 1.0f, 0.0f));
      // shader.SetMatrix4fv("model", glm::value_ptr(model));

      view = glm::lookAt(camera_pos, camera_pos + camera_front, up);

      shader.SetMatrix4fv("view", glm::value_ptr(view));
      shader.SetMatrix4fv("projection", glm::value_ptr(projection));

      // bind textures
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, texture1);
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, texture2);

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
      }

      // glDrawArrays(GL_TRIANGLES, 0, 6);
      // glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

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

  float camera_speed = 3.0f * delta_time;
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
  } else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    camera_pos += camera_speed * camera_front;
  } else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    camera_pos -= camera_speed * camera_front;
  } else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    camera_pos -= camera_speed * glm::normalize(
        glm::cross(camera_front, up));
  } else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    camera_pos += camera_speed * glm::normalize(
        glm::cross(camera_front, up));
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
  float sensitivity = 0.05;

  last_x = xpos;
  last_y = ypos;

  yaw -= sensitivity * xoffset;
  pitch += sensitivity * yoffset;

  if (pitch > 89.0f) {
    pitch = 89.0f;
  } else if (pitch < - 89.0f) {
    pitch = -89.0f;
  }

  glm::vec3 front;
  front.x = cos(glm::radians(pitch)) * cos(glm::radians(yaw));
  front.y = sin(glm::radians(pitch));
  front.z = - cos(glm::radians(pitch)) * sin(glm::radians(yaw));
  camera_front = glm::normalize(front);

  // std::cout << "mmmmmmmmmmmmmmmmmmmmm" << std::endl;
  // std::cout << "mouse x, y = " << xpos << ", " << ypos << std::endl;
  // std::cout << "xoff, yoff = " << xoffset << ", " << yoffset << std::endl;
  // std::cout << "pitch, yaw = " << pitch << ", " << yaw << std::endl;
  // std::cout << "camera_front = " << glm::to_string(camera_front) << std::endl;
}
