// Copyright 2018 Pavlo

#include "cv_gl/mesh.h"


Mesh::Mesh(const std::vector<Vertex>& vertices,
     const std::vector<unsigned int>& indices,
     const std::vector<Texture>& textures) {

  this->vertices = vertices;
  this->indices = indices;
  this->textures = textures;

  SetupMesh();

}

Mesh::Mesh(const Mesh& mesh) {
  std::cout << "mesh (COPY CON) = ";
  mesh.write();
  std::cout << std::endl;
}

Mesh Mesh::operator=(const Mesh& mesh) {
  std::cout << "mesh (COPY ASS) = ";
  this->write();
  std::cout << std::endl;
  return Mesh(mesh);
}

void
Mesh::Draw(const Shader& shader) {
  // draw mesh
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}

void
Mesh::SetupMesh() {

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
      &vertices[0], GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
      &indices[0], GL_STATIC_DRAW);

  // position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)0);
  glEnableVertexAttribArray(0);

  //color attribute
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, normal)));
  glEnableVertexAttribArray(1);

  //tex coords
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, tex_coords)));
  glEnableVertexAttribArray(2);

  // Ubind buffer
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  std::cout << "mesh (SETUP) = ";
  this->write();
  std::cout << std::endl;

}

Mesh::~Mesh() {
  std::cout << "mesh (DESTRUCTOR) = ";
  this->write();
  std::cout << std::endl;
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  glDeleteBuffers(1, &ebo_);
}

std::ostream& operator<<(std::ostream& os, const Vertex& vertex) {
  os << glm::to_string(vertex.position) << " "
    << glm::to_string(vertex.normal) << " "
    << glm::to_string(vertex.tex_coords);
  return os;
}

std::ostream& Mesh::write(std::ostream& os) const {
  os << "vao_: " << vao_ << ", vbo_: " << vbo_ << ", ebo: " << ebo_;
  return os;
}
std::ostream& operator<<(std::ostream& os, const Mesh& mesh) {
  return mesh.write(os);
}
