// Copyright 2018 Pavlo
#ifndef CV_GL_MESH_H_
#define CV_GL_MESH_H_

#include <glm/glm.hpp>

#include <opencv2/opencv.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include <vector>
#include <iostream>

#include "cv_gl/shader.h"

enum class MeshType {LINES, TRIANGLES, POINTS};

enum class TextureType {AMBIENT, DIFFUSE, SPECULAR};
std::string TextureTypeName(TextureType texture_type);

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 tex_coords;
  glm::vec3 color;
  glm::vec3 color_tl;
  glm::vec3 color_tr;
  glm::vec3 color_bl;
  glm::vec3 color_br;
};

struct Texture {
  Texture() : type(TextureType::DIFFUSE) {}
  unsigned int id;
  TextureType type;
  std::string path;
  unsigned int width;
  unsigned int height;
};


// Texture::texture_type_name = {""};

struct Material {
  Material(const glm::vec4 amb_color = {0.15, 0.15, 0.15, 1.0},
           const glm::vec4 diff_color = {0.5, 0.5, 0.5, 1.0})
      : ambient_color(amb_color), 
        diffuse_color(diff_color), 
        ambient_transparent(0),
        use_vertex_color(0) {};
  glm::vec4 ambient_color;
  glm::vec4 diffuse_color;
  int ambient_transparent;
  int use_vertex_color;
  std::vector<Texture> textures;
  bool IsTransparent() const { return ambient_transparent > 0; }
};

class Mesh {
 public:
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  Material material;
  // std::vector<Texture> textures;

  

  void SetMeshType(const MeshType& mesh_type) { mesh_type_ = mesh_type; }
  MeshType GetMeshType() const { return mesh_type_; }

  Mesh(const std::vector<Vertex>& vertices,
       const std::vector<unsigned int>& indices,
       const std::vector<Texture>& textures);
  Mesh(const std::vector<Vertex>& vertex,
       const std::vector<unsigned int>& indices,
       const Material& material);
  Mesh(const Mesh& mesh);
  Mesh operator=(const Mesh& mesh);

  glm::vec3 GetCenterPoint() { return center_point_; }

  ~Mesh();

  void Draw(const std::shared_ptr<Shader>& shader);

  static Texture TextureFromFile(const std::string& path, const std::string& directory = "",
      const bool flip = false, const float scale_f = 1.0);
      
  static Texture TextureFromMat(const cv::Mat& img, const bool flip = false);

  std::ostream& print(std::ostream& os = std::cout) const;


private:
  unsigned int vao_, vbo_, ebo_;
  MeshType mesh_type_;
  glm::vec3 center_point_;
  void SetupMesh();

  static int count_;


};



std::ostream &operator<<(std::ostream &os, const Vertex &vertex);
std::ostream &operator<<(std::ostream &os, const std::vector<Vertex>& vertices);
std::ostream &operator<<(std::ostream &os, const Texture &texture);
std::ostream &operator<<(std::ostream &os, const Material &material);
std::ostream &operator<<(std::ostream &os, const Mesh &mesh);
std::ostream &operator<<(std::ostream &os, const std::shared_ptr<Mesh> &mesh);

#endif  // CV_GL_MESH_H_
