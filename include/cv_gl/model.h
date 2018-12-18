// Copyright 2018 Pavlo
#ifndef CV_GL_MODEL_H_
#define CV_GL_MODEL_H_

#include <glad/glad.h>

#include "cv_gl/shader.h"
#include "cv_gl/mesh.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// #define STB_IMAGE_IMPLEMENTATION
// #include <stb_image.h>

#include <memory>


unsigned int TextureFromFile(const char* path, const std::string& directory,
    bool gamma = false);


class Model {
public:
  Model(const std::string& path) {
    LoadModel(path);
  }
  void Draw(const std::shared_ptr<Shader>& shader);
  std::vector<std::shared_ptr<Mesh> > GetMeshes() { return meshes_; }
  void print(std::ostream &os = std::cout) const;


  std::vector<std::shared_ptr<Mesh> > meshes_;

private:
  std::string directory_;
  std::vector<Texture> textures_loaded_;

  void LoadModel(const std::string& path);
  void ProcessNode(const aiNode *node, const aiScene *scene);
  std::shared_ptr<Mesh> ProcessMesh(const aiMesh *mesh, const aiScene *scene);
  std::vector<Texture> LoadMaterialTextures(const  aiMaterial *material,
      const aiTextureType type, const TextureType texture_type);
};

glm::vec4 to_glm_color4(const aiColor4D& c) {
  return glm::vec4(c.r, c.g, c.b, c.a);
}

void
Model::LoadModel(const std::string& path) {
  Assimp::Importer importer;

  const aiScene *scene = importer.ReadFile(path,
      aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals /*aiProcess_GenSmoothNormals*/);

  // std::cout << "mNumMaterials = " << scene->mNumMaterials << std::endl;

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    std::cout << "ERROR:ASSIMP:" << importer.GetErrorString() << std::endl;
    return;
  }
  directory_ = path.substr(0, path.find_last_of('/'));
  // std::cout << "directory_ = " << directory_ << std::endl;
  ProcessNode(scene->mRootNode, scene);
  std::cout << "MODEL LOADED: " << path << std::endl;
}

void
Model::ProcessNode(const aiNode *node, const aiScene *scene) {
  // std::cout << "Process NODE: m=" << node->mNumMeshes
      // << ", c=" << node->mNumChildren << std::endl;
  for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    meshes_.emplace_back(ProcessMesh(mesh, scene));
  }
  // std::cout << "Meshes.size = " << meshes_.size() << std::endl;

  for (unsigned int i = 0; i < node->mNumChildren; ++i) {
    this->ProcessNode(node->mChildren[i], scene);
  }
}

std::shared_ptr<Mesh>
Model::ProcessMesh(const aiMesh *mesh, const aiScene *scene) {
  // std::cout << "Process MESH" << std::endl;
  std::vector<Vertex> vertices;
  std::vector<Texture> textures;
  std::vector<unsigned int> indices;
  for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
    // process vertex
    Vertex vertex;

    vertex.position = {mesh->mVertices[i].x, mesh->mVertices[i].y,
        mesh->mVertices[i].z};

    if (mesh->HasNormals()) {
      // std::cout << "HAS NORMALS!!!!!!!!!!!!!!!!!!" << std::endl;
      vertex.normal = {mesh->mNormals[i].x, mesh->mNormals[i].y,
                       mesh->mNormals[i].z};
    }
    

    // std::cout << "numUVChannels = " << mesh->GetNumUVChannels() << std::endl;

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

  // std::cout << "v: " << vertices.size() << std::endl;

  // process indices

  // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
  for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace face = mesh->mFaces[i];
    // retrieve all indices of the face and store them in the indices vector
    for(unsigned int j = 0; j < face.mNumIndices; j++) {
      indices.push_back(face.mIndices[j]);
    }
  }

  // std::cout << "i: " << indices.size() << std::endl;


  // process material
  // if (mesh->mMaterialIndex > 0) {

    // std::cout << "mMaterialIndex = " << mesh->mMaterialIndex << std::endl;

    aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
    std::vector<Texture> diffuse_maps = LoadMaterialTextures(material,
        aiTextureType_DIFFUSE, TextureType::DIFFUSE);
    // std::cout << "difuse_maps = " << diffuse_maps.size() << std::endl;
    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());

    std::vector<Texture> specular_maps = LoadMaterialTextures(material,
        aiTextureType_SPECULAR, TextureType::SPECULAR);
    // std::cout << "specular_maps = " << specular_maps.size() << std::endl;



    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());
  // }

  // std::cout << "texturesCount = " << textures.size() << std::endl;
  // std::cin.ignore();

    // Material mat;
    // std::cin.ignore();

    Material mat;
    mat.textures = textures;
    // std::cout << "<<<< mat = " << mat << std::endl;
    // for (auto t : mat.textures) {
    //   std::cout << " --- " << t << std::endl;
    // }

    aiColor4D ambient;
    if (AI_SUCCESS == aiGetMaterialColor(material, AI_MATKEY_COLOR_AMBIENT, &ambient))
    {
      glm::vec4 ac = to_glm_color4(ambient);
      mat.ambient_color = ac;
      // std::cout << " ========================== ambient = " << glm::to_string(ac) << std::endl;
      // std::cin.ignore();
    }

    aiColor4D diffuse;
    if (AI_SUCCESS == aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse)) {
      // color4_to_float4(&diffuse, c);
      glm::vec4 dc = to_glm_color4(diffuse);
      // std::cout << " ========================== diffuse = " << glm::to_string(dc) << std::endl;
      mat.diffuse_color = dc;
    }

    auto m = std::make_shared<Mesh>(vertices, indices, mat);
    
    // std::cout << "<<<<< m = " << m << std::endl;

    return m;
}

// TODO: Look At http://www.lighthouse3d.com/cg-topics/code-samples/importing-3d-models-with-assimp/

std::vector<Texture>
Model::LoadMaterialTextures(const aiMaterial *material,
    const aiTextureType type, const TextureType texture_type) {
  std::vector<Texture> textures;

  // std::cout << "textureCount = " << material->GetTextureCount(type) << " for aiTextureType = " << type << std::endl;

  for(unsigned int i = 0; i < material->GetTextureCount(type); ++i) {
    aiString str;
    material->GetTexture(type, i, &str);
    // std::cout << "Got texture: " << std::string(str.C_Str())
    //     << " (" << type_name << ")" << std::endl;

    bool skip = false;
    for (unsigned int j = 0; j < textures_loaded_.size(); ++j) {
      if (std::strcmp(textures_loaded_[j].path.data(), str.C_Str()) == 0) {
        textures.emplace_back(textures_loaded_[j]);
        skip = true;
        // std::cout << "Skip texture loading: " << textures_loaded_[j].path << std::endl;
        break;
      }
    }

    if (!skip) {
      Texture texture = Mesh::TextureFromFile(std::string(str.C_Str()), directory_);
      texture.type = texture_type;
      texture.path = std::string(str.C_Str());
      textures.emplace_back(texture);
      textures_loaded_.emplace_back(texture);
    }
  }
  return textures;
}

void
Model::Draw(const std::shared_ptr<Shader>& shader) {
  // std::cout << "MODEL:DRAW:MESHES = " << meshes_.size() << std::endl;
  for (unsigned int i = 0; i < meshes_.size(); ++i) {
    meshes_[i]->Draw(shader);
  }
}

void
Model::print(std::ostream& os) const {
  os << "Model: ";
  os << "meshes.size = " << meshes_.size() << ", ";
  os << "textures_loaded.size = " << textures_loaded_.size();

  os << std::endl;
  os << "  meshes: " << std::endl;
  for (const std::shared_ptr<Mesh>& m : meshes_) {
    os << "  > " << m << std::endl;
  }

  // os << std::endl;
  os << "  textures_loaded: " << std::endl;
  for (const Texture &tl : textures_loaded_)
  {
    os << "  > " << tl << std::endl;
  }
}



std::ostream& operator<<(std::ostream& os, const Model& model) {
  model.print(os);
  return os;
}

std::ostream& operator<<(std::ostream& os, const std::shared_ptr<Model>& model_ptr) {
  model_ptr->print(os);
  return os;
}


#endif  // CV_GL_MODEL_H_
