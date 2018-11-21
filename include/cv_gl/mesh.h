// Copyright 2018 Pavlo
#ifndef CV_GL_MESH_H_
#define CV_GL_MESH_H_

#include <glm/glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include <vector>
#include <iostream>

#include "cv_gl/shader.h"

enum class MeshType {LINES, TRIANGLES};

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 tex_coords;
};

struct Texture {
  unsigned int id;
  std::string type;
  std::string path;
};

class Mesh {
 public:
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;

  void SetMeshType(const MeshType& mesh_type) { mesh_type_ = mesh_type; }

  Mesh(const std::vector<Vertex>& vertices,
       const std::vector<unsigned int>& indices,
       const std::vector<Texture>& textures);
  Mesh(const Mesh& mesh);
  Mesh operator=(const Mesh& mesh);

  ~Mesh();

  void Draw(const std::shared_ptr<Shader>& shader);
  std::ostream& print(std::ostream& os = std::cout) const;



private:
  unsigned int vao_, vbo_, ebo_;
  MeshType mesh_type_;
  void SetupMesh();



};


std::ostream& operator<<(std::ostream& os, const Vertex& vertex);
std::ostream& operator<<(std::ostream& os, const Mesh& mesh);
std::ostream& operator<<(std::ostream& os, const std::shared_ptr<Mesh>& mesh);



#endif  // CV_GL_MESH_H_
