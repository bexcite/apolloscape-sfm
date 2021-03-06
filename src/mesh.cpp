// Copyright 2018 Pavlo

#include "cv_gl/mesh.h"

#include <opencv2/opencv.hpp>

#include <glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stb_image_resize.h>


std::string TextureTypeName(TextureType texture_type) {
  switch(texture_type) {
    case TextureType::AMBIENT: return "texture_ambient";
    case TextureType::DIFFUSE: return "texture_diffuse";
    case TextureType::SPECULAR: return "texture_specular";
    default: return "NONE";
  }
};

int Mesh::count_ = 0;

Mesh::Mesh(const std::vector<Vertex>& vertices,
     const std::vector<unsigned int>& indices,
     const std::vector<Texture>& textures) : mesh_type_(MeshType::TRIANGLES) {
  this->vertices = vertices;
  this->indices = indices;
  this->material.textures = textures;
  SetupMesh();
}

Mesh::Mesh(const std::vector<Vertex> &vertices,
           const std::vector<unsigned int> &indices,
           const Material &material) : mesh_type_(MeshType::TRIANGLES) {
  this->vertices = vertices;
  this->indices = indices;
  this->material = material;
  SetupMesh();
}

Mesh::Mesh(const Mesh& mesh) {
  std::cout << "mesh (COPY CON) = ";
  mesh.print();
  std::cout << std::endl;
}

Mesh Mesh::operator=(const Mesh& mesh) {
  std::cout << "mesh (COPY ASS) = ";
  this->print();
  std::cout << std::endl;
  return Mesh(mesh);
}

void
Mesh::Draw(const std::shared_ptr<Shader>& shader) {

  // setup textures
  unsigned int diffuse_num = 1;
  unsigned int specular_num = 1;
  unsigned int ambient_num = 1;
  for (unsigned int i = 0; i < material.textures.size(); ++i) {
    glActiveTexture(GL_TEXTURE0 + i);

    std::string number_str;
    unsigned int number;
    TextureType type = material.textures[i].type;
    std::string type_name = TextureTypeName(type); //material.textures[i].type;
    
    if (type == TextureType::DIFFUSE) {
      number = diffuse_num++;
      number_str = std::to_string(number);
    } else if (type == TextureType::SPECULAR) {
      number = specular_num++;
      number_str = std::to_string(number);
    } else if (type == TextureType::AMBIENT) {
      number = ambient_num++;
      number_str = std::to_string(number);
    }

    // TODO!!!!
    shader->SetInt("material." + type_name, number);
    // std::cout << "material." + type_name << " = " << number << std::endl;

    glBindTexture(GL_TEXTURE_2D, material.textures[i].id);
    shader->SetInt("material." + type_name + number_str, i);
  }

  if (material.textures.empty()) {
    shader->SetInt("material.texture_diffuse", 0);
    shader->SetInt("material.texture_ambient", 0);
  }

  shader->SetInt("material.texture_ambient_transparent", material.ambient_transparent);
  shader->SetInt("material.use_vertex_color", material.use_vertex_color);

  // shader->SetInt("material.texture_ambient_transparent", 0);

  shader->SetVector4fv("material.ambient", glm::value_ptr(material.ambient_color));
  shader->SetVector4fv("material.diffuse", glm::value_ptr(material.diffuse_color));

  // if (!material.textures.empty()) {
  //   shader->SetInt("material.texture", material.textures.size());
  // }

  // draw mesh
  glBindVertexArray(vao_);

  if (mesh_type_ == MeshType::TRIANGLES) {
    if (!indices.empty()) {
      glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    } else {
      glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    }
  } else if (mesh_type_ == MeshType::LINES) {
    if (!indices.empty()) {
      glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
    } else {
      glDrawArrays(GL_LINES, 0, vertices.size());
    }
  } else if (mesh_type_ == MeshType::POINTS) {
    if (!indices.empty()) {
      glDrawElements(GL_POINTS, indices.size(), GL_UNSIGNED_INT, 0);
    } else {
      glDrawArrays(GL_POINTS, 0, vertices.size());
    }
  }

  glBindVertexArray(0);
}

void
Mesh::SetupMesh() {

  // std::cout << "Mesh: Setup (" << (++count_) << ")" << std::endl;

  glGenVertexArrays(1, &vao_);
  // std::cout << "Mesh: VAO (" << vao_ << ")" << std::endl;
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

  //color
  glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, color)));
  glEnableVertexAttribArray(3);

  //color_tl
  glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, color_tl)));
  glEnableVertexAttribArray(4);
  //color_tr
  glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, color_tr)));
  glEnableVertexAttribArray(5);
  //color_bl
  glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, color_bl)));
  glEnableVertexAttribArray(6);
  //color_br
  glVertexAttribPointer(7, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      (void*)(offsetof(Vertex, color_br)));
  glEnableVertexAttribArray(7);

  // Ubind buffer
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  // std::cout << "mesh (SETUP) = ";
  // this->write();
  // std::cout << std::endl;

  // Calc center_point
  if (!vertices.empty()) {
    glm::vec3 cp(0.0f);
    for (auto& v : vertices) {
      cp += v.position;
    }
    cp /= vertices.size();
    center_point_ = cp;
    // std::cout << "center_point = " << glm::to_string(center_point_) << std::endl;
  }

}


Mesh::~Mesh() {
  // std::cout << "mesh (DESTRUCTOR) " << (count_--) << std::endl;
  // this->write();
  // std::cout << std::endl;
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  glDeleteBuffers(1, &ebo_);
}

Texture Mesh::TextureFromFile(const std::string& path, const std::string& directory,
    const bool flip, const float scale_f) {

  Texture texture;

  std::string filename = std::string(path);
  if (!directory.empty()) {
    filename = directory + "/" + filename;
  }
  // std::cout << "filename = " << filename << std::–endl;

  // unsigned int texture_id;
  glGenTextures(1, &texture.id);

  stbi_set_flip_vertically_on_load(flip);

  int width, height, nrComponents;
  unsigned char* data = stbi_load(filename.c_str(), &width, &height,
      &nrComponents, 0);

  texture.width = width;
  texture.height = height;

  // std::cout << "width = " << width << std::endl;
  // std::cout << "height = " << height << std::endl;

  if (data) {
    GLenum format;
    if (nrComponents == 1) {
      format = GL_RED;
    } else if (nrComponents == 3) {
      format = GL_RGB;
    } else if (nrComponents == 4) {
      format = GL_RGBA;
    }

    // Scale factor
    // float scale_f = 4.5;

    // TODO: Don't need to scale if it 1.0 :)) just remark for future self
    int new_width = static_cast<int>((width * scale_f) / 4) * 4;  // sizes not aligned to a word (4 bytes) are not correctlyrendered in opengl
    int new_height = static_cast<int>((height * scale_f) / 4) * 4;

    texture.width = new_width;
    texture.height = new_height;

    unsigned char* data_resize = (unsigned char *) malloc(new_width * new_height * nrComponents * sizeof(unsigned char));

    // std::cout << "nrComponents = " << nrComponents << std::endl;
    // std::cout << "new_width = " << new_width << std::endl;
    // std::cout << "new_height = " << new_height << std::endl;
    stbir_resize_uint8(data, width, height, 0, data_resize, new_width, new_height, 0, nrComponents);

    glBindTexture(GL_TEXTURE_2D, texture.id);

    // glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format,
    //     GL_UNSIGNED_BYTE, data);

    glTexImage2D(GL_TEXTURE_2D, 0, format, new_width, new_height, 0, format,
        GL_UNSIGNED_BYTE, data_resize);


    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
        GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
        GL_LINEAR);

    stbi_image_free(data);
    free(data_resize);

  } else {
    std::cout << "Texture failed to load at path: " << path << std::endl;
    stbi_image_free(data);
  }

  return texture;

}

Texture Mesh::TextureFromMat(const cv::Mat& mat, const bool flip) {
  Texture texture;

  // unsigned int texture_id;
  glGenTextures(1, &texture.id);


  int width, height, nrComponents;

  texture.width = mat.size().width;
  texture.height = mat.size().height;
  nrComponents = mat.channels();

  GLenum format;

  if (nrComponents == 1) {
    format = GL_RED;
  } else if (nrComponents == 3) {
    format = GL_BGR;
  } else if (nrComponents == 4) {
    format = GL_BGRA;
  }

  // std::cout << "TfM: w, h, nrC = " << texture.width << ", " << texture.height << ", " << nrComponents << std::endl;

  glBindTexture(GL_TEXTURE_2D, texture.id);

  // glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format,
  //     GL_UNSIGNED_BYTE, data);

  // TODO(FIX ME): quick fix! 
  double scale_f = 1.0;
  int new_width = static_cast<int>((texture.width * scale_f) / 4) * 4;  // sizes not aligned to a word (4 bytes) are not correctlyrendered in opengl
  int new_height = static_cast<int>((texture.height * scale_f) / 4) * 4;
  texture.width = new_width;
  texture.height = new_height;
  cv::Mat mat_resized;
  cv::resize(mat, mat_resized, cv::Size(new_width, new_height));

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture.width, texture.height, 0, format,
      GL_UNSIGNED_BYTE, mat_resized.data);

  glGenerateMipmap(GL_TEXTURE_2D);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
      GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
      GL_LINEAR);

  // glBindTexture(GL_TEXTURE_2D, 0);

  return texture;
}

std::ostream& operator<<(std::ostream& os, const Vertex& vertex) {
  os << glm::to_string(vertex.position) << " "
    << glm::to_string(vertex.normal) << " "
    << glm::to_string(vertex.tex_coords);
  return os;
}

std::ostream &operator<<(std::ostream &os, const std::vector<Vertex> &vertices) {
  os << std::endl;
  for (auto v : vertices) {
    os << "  -> " << v << std::endl;
  }
  return os;
}

std::ostream &operator<<(std::ostream &os, const Material &material) {
  os << "Material: diffuse_color = " << glm::to_string(material.diffuse_color) << ", ";
  os << "ambient_color = " << glm::to_string(material.ambient_color) << ", ";
  os << "ambient_transparent = " << material.ambient_transparent << ", ";
  os << "textures.size = " << material.textures.size();
  return os;
}

std::ostream &operator<<(std::ostream &os, const Texture &texture) {
  os << "Texture: " << texture.id << ", " << TextureTypeName(texture.type) << ", " << texture.path;
  return os;
}

std::ostream& Mesh::print(std::ostream& os) const {
  os << "Mesh: ";
  os << "vao_:" << vao_ << ", vbo_:" << vbo_ << ", ebo:" << ebo_;
  os << " vertices.size = " << this->vertices.size() << ", ";
  os << "indices.size = " << this->indices.size() << ", ";
  // os << "textures.size = " << this->textures.size() << ", ";
  os << "material = " << this->material;
  return os;
}
std::ostream& operator<<(std::ostream& os, const Mesh& mesh) {
  // os << " [CALL BY REF] ";
  return mesh.print(os);
}
std::ostream& operator<<(std::ostream& os, const std::shared_ptr<Mesh>& mesh) {
  // os << " [CALL BY SHARED] ";
  return mesh->print(os);
}
