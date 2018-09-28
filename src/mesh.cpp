// Copyright 2018 Pavlo

#include "cv_gl/mesh.h"


Mesh::Mesh(const std::vector<Vertex>& vertices,
     const std::vector<unsigned int>& indices,
     const std::vector<Texture>& textures) {

  this->vertices = vertices;
  this->indices = indices;
  this->textures = textures;

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


}
