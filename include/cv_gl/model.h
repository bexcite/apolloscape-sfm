// Copyright 2018 Pavlo
#ifndef CV_GL_MODEL_H_
#define CV_GL_MODEL_H_

#include <glad/glad.h>

#include "cv_gl/shader.h"
#include "cv_gl/mesh.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>



unsigned int TextureFromFile(const char* path, const std::string& directory,
    bool gamma = false);


class Model {
public:
  Model(const std::string& path) {
    LoadModel(path);
  }
  void Draw(const Shader& shader);
  std::vector<Mesh> meshes_;
private:
  std::string directory_;
  std::vector<Texture> textures_loaded_;

  void LoadModel(const std::string& path);
  void ProcessNode(const aiNode *node, const aiScene *scene);
  Mesh ProcessMesh(const aiMesh *mesh, const aiScene *scene);
  std::vector<Texture> LoadMaterialTextures(const  aiMaterial *material,
      const aiTextureType type, const std::string& type_name);
};



void
Model::LoadModel(const std::string& path) {
  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(path,
      aiProcess_Triangulate | aiProcess_FlipUVs);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    std::cout << "ERROR:ASSEMP:" << importer.GetErrorString() << std::endl;
    return;
  }
  directory_ = path.substr(0, path.find_last_of('/'));
  std::cout << "directory_ = " << directory_ << std::endl;
  ProcessNode(scene->mRootNode, scene);
  std::cout << "MODEL LOADED" << std::endl;
}

void
Model::ProcessNode(const aiNode *node, const aiScene *scene) {
  std::cout << "Process NODE: m=" << node->mNumMeshes
      << ", c=" << node->mNumChildren << std::endl;
  for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    meshes_.emplace_back(ProcessMesh(mesh, scene));
  }
  std::cout << "Meshes.size = " << meshes_.size() << std::endl;

  for (unsigned int i = 0; i < node->mNumChildren; ++i) {
    this->ProcessNode(node->mChildren[i], scene);
  }
}

Mesh
Model::ProcessMesh(const aiMesh *mesh, const aiScene *scene) {
  std::cout << "Process MESH" << std::endl;
  std::vector<Vertex> vertices;
  std::vector<Texture> textures;
  std::vector<unsigned int> indices;
  for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
    // process vertex
    Vertex vertex;

    vertex.position = {mesh->mVertices[i].x, mesh->mVertices[i].y,
        mesh->mVertices[i].z};

    vertex.normal = {mesh->mNormals[i].x, mesh->mNormals[i].y,
        mesh->mNormals[i].z};

    if (mesh->mTextureCoords[0]) {
      vertex.tex_coords = {
        mesh->mTextureCoords[0][i].x,
        mesh->mTextureCoords[0][i].y
      };
    } else {
      vertex.tex_coords = {0.0f, 0.0f};
    }

    // std::cout << "v: " << vertex << std::endl;

    vertices.emplace_back(vertex);
  }

  std::cout << "v: " << vertices.size() << std::endl;

  // process indices

  // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
  for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace face = mesh->mFaces[i];
    // retrieve all indices of the face and store them in the indices vector
    for(unsigned int j = 0; j < face.mNumIndices; j++) {
      indices.push_back(face.mIndices[j]);
    }
  }

  std::cout << "i: " << indices.size() << std::endl;


  // process material
  if (mesh->mMaterialIndex > 0) {
    aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    std::vector<Texture> diffuse_maps = LoadMaterialTextures(material,
        aiTextureType_DIFFUSE, "texture_diffuse");
    std::cout << "difuse_maps = " << diffuse_maps.size() << std::endl;
    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());

    std::vector<Texture> specular_maps = LoadMaterialTextures(material,
        aiTextureType_SPECULAR, "texture_specular");
    std::cout << "specular_maps = " << specular_maps.size() << std::endl;
    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());
  }


  return Mesh(vertices, indices, textures);
}

std::vector<Texture>
Model::LoadMaterialTextures(const aiMaterial *material,
    const aiTextureType type, const std::string& type_name) {
  std::vector<Texture> textures;

  for(unsigned int i = 0; i < material->GetTextureCount(type); ++i) {
    aiString str;
    material->GetTexture(type, i, &str);
    std::cout << "Got texture: " << std::string(str.C_Str())
        << " (" << type_name << ")" << std::endl;

    bool skip = false;
    for (unsigned int j = 0; j < textures_loaded_.size(); ++j) {
      if (std::strcmp(textures_loaded_[j].path.data(), str.C_Str()) == 0) {
        textures.emplace_back(textures_loaded_[j]);
        skip = true;
        std::cout << "Skip texture loading: " << textures_loaded_[j].path
            << std::endl;
        break;
      }
    }

    if (!skip) {
      Texture texture;
      texture.id = TextureFromFile(str.C_Str(), directory_);
      texture.type = type_name;
      texture.path = std::string(str.C_Str());
      textures.emplace_back(texture);
      textures_loaded_.emplace_back(texture);
    }
  }
  return textures;
}

void
Model::Draw(const Shader& shader) {
  // std::cout << "MODEL:DRAW:MESHES = " << meshes_.size() << std::endl;
  for (unsigned int i = 0; i < meshes_.size(); ++i) {
    meshes_[i].Draw(shader);
  }
}

unsigned int TextureFromFile(const char* path, const std::string& directory,
    bool gamma) {

  std::string filename = std::string(path);
  filename = directory + "/" + filename;
  std::cout << "filename = " << filename << std::endl;

  unsigned int texture_id;
  glGenTextures(1, &texture_id);

  int width, height, nrComponents;
  unsigned char* data = stbi_load(filename.c_str(), &width, &height,
      &nrComponents, 0);

  if (data) {
    GLenum format;
    if (nrComponents == 1) {
      format = GL_RED;
    } else if (nrComponents == 3) {
      format = GL_RGB;
    } else if (nrComponents == 4) {
      format = GL_RGBA;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format,
        GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
        GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
        GL_LINEAR);

    stbi_image_free(data);

  } else {
    std::cout << "Texture failed to load at path: " << path << std::endl;
    stbi_image_free(data);
  }

  return texture_id;

}


#endif  // CV_GL_MODEL_H_
