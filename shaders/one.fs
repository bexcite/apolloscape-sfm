#version 330 core
out vec4 FragColor;

in vec4 vertexColor;
in vec2 TexCoord;

uniform vec4 ourColor;

uniform sampler2D texture1;
uniform sampler2D texture2;

void main() {
  // FragColor = vertexColor * ourColor;
  vec4 col1 = texture(texture1, TexCoord);
  vec4 col2 = texture(texture2, TexCoord);
  FragColor = mix(col1, col2, ourColor.x) * 0.8 + 0.2;
  // FragColor = ourColor;
  // FragColor = vec4(1.0f);
}
