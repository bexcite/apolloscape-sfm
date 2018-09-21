// Copyright Pavlo 2017

// System Headers
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// Standard Headers
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <iostream>


// Define Some Constants
const int kWindowWidth = 1280;
const int kWindowHeight = 800;

void processInput(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);

const char* vertexShaderSource = "#version 330 core\n"
  "layout (location = 0) in vec3 aPos;\n"
  "layout (location = 1) in vec3 aColor;\n"
  "out vec4 vertexColor;\n"
  "void main() {\n"
  "  gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0f);\n"
  "  vertexColor = vec4(aColor, 1.0f);\n"
  "}\n\0";

const char* fragmentShaderSource = "#version 330 core\n"
  "out vec4 FragColor;\n"
  "in vec4 vertexColor;"
  "uniform vec4 ourColor;\n"
  "void main() {\n"
  "  //FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
  "  FragColor = vertexColor;\n"
  "  //FragColor = ourColor;\n"
  "}\n\0";



int main(int argc, char* argv[]) {
  std::cout << "Hello world!" << std::endl;



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
  fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));
  fprintf(stderr, "OpenGL Vendor %s\n", glGetString(GL_VENDOR));
  fprintf(stderr, "OpenGL Renderer %s\n", glGetString(GL_RENDERER));

  //  Create vertexShader
  int success;
  char infoLog[512];

  int vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
  glCompileShader(vertexShader);
  // check for shader compile errors
  glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
    std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED" << infoLog
        << std::endl;
  }

  // Create fragmentShader
  int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
  glCompileShader(fragmentShader);
  // check for errors
  glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
    std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_ERROR\n" << infoLog
        << std::endl;
  }

  // link shader into program
  int shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
    std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog
        << std::endl;
  }
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  // setup vertex data
  float vertices[] = {
    0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f,   // top right 0
    0.5f, -0.5f, 0.0f, 0.0f, 1.0f, 0.0f,  // bottom right 1
    -0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 1.0f, // bottom left 2
    -0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f,   // top left 3
    0.0f, 0.75f, 0.0f, 0.0f, 1.0f, 0.0f,   // center top 4
    0.0f, -0.75f, 0.0f, 0.0f, 0.0f, 1.0f   // center bottom 5
  };

  // unsigned int indices[] = {
  //   0, 1, 3,
  //   1, 2, 3,
  //   0, 3, 4,
  //   1, 2, 5
  // };

  unsigned int indices[] = {
    0, 1, 3,
    1, 2, 3,
    0, 3, 4,
    1, 2, 5
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
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
      (void*)0);
  glEnableVertexAttribArray(0);

  //color attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
      (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  float timeSince = 0;
  unsigned int frame = 0;

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

      int vertexColorLocation = glGetUniformLocation(shaderProgram, "ourColor");
      glUseProgram(shaderProgram);
      glUniform4f(vertexColorLocation, 0.0f, greenValue, 0.0f, 1.0f);

      glBindVertexArray(VAO);

      // glDrawArrays(GL_TRIANGLES, 0, 6);
      glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, 0);

      // Flip Buffers and Draw
      glfwSwapBuffers(window);
      glfwPollEvents();
  }

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
