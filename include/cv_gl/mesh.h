// Copyright 2018 Pavlo
#ifndef CV_GL_MESH_H_
#define CV_GL_MESH_H_

#include <glm/glm.hpp>

#include <vector>

#include "cv_gl/shader.h"

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 tex_coords;
};

struct Texture {
  unsigned int id;
  std::string type;
};

class Mesh {
 public:
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;

  Mesh(const std::vector<Vertex>& vertices,
       const std::vector<unsigned int>& indices,
       const std::vector<Texture>& textures);

  void Draw(const Shader& shader);

private:
  unsigned int vao_, vbo_, ebo_;
  void SetupMesh();


};

#endif  // CV_GL_MESH_H_
