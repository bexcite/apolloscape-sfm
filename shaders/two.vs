#version 330 core

layout (location = 0) in vec3 aPos;
// layout (location = 1) in vec3 aColor;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in vec3 aColor;

out vec4 VertexColor;
out vec2 TexCoord;
out vec3 FragPos;
out vec3 Normal;

// TODO: Change to arrays (GLSL)
out vec3 LightDir1;
out vec3 LightDir2;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;



void main() {
  vec4 pos = vec4(aPos, 1.0f);
  // gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0f);
  gl_Position = projection * view * model * pos;
  gl_PointSize = 2.0;

  vec3 lightDir1 = normalize(vec3(-1.0, -0.3, 0.6));

  vec3 lightDir2 = vec3(-lightDir1.x, -lightDir1.y, -lightDir1.z);

  mat3 viewNormal = mat3(transpose(inverse(view)));

  LightDir1 = normalize(viewNormal * lightDir1);
  LightDir2 = normalize(viewNormal * lightDir2);

//   vertexColor = vec4(0.5f, 0.0f, 0.0f, 1.0f);
  TexCoord = aTexCoord;

  FragPos = vec3(view * model * pos);

  // need to transform
  Normal = normalize(mat3(transpose(inverse(view * model))) * aNormal);

  VertexColor = vec4(aColor, 1.0);
}
