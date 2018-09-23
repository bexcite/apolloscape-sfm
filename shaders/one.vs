#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec2 aTexCoord;

out vec4 vertexColor;
out vec2 TexCoord;

void main() {
  gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0f);
  vertexColor = vec4(aColor, 1.0f);
  TexCoord = aTexCoord;
}
