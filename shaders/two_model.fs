#version 330 core


struct Material {
    vec4 diffuse;
    vec4 ambient;
    int texture;
};

out vec4 FragColor;

in vec2 TexCoord;
in vec3 FragPos;
in vec3 Normal;
in vec3 LightDir1;
in vec3 LightDir2;

uniform sampler2D texture_diffuse1;

uniform Material material;

// uniform vec4 diffuse_color;

void main() {
  // FragColor = vertexColor * ourColor;
  // vec4 col1 = texture(texture1, TexCoord);
  // vec4 col2 = texture(texture2, TexCoord);
  // FragColor = mix(col1, col2, ourColor.x) * 0.8 + 0.2;

  float diff1 = max(dot(Normal, LightDir1), 0.0);
  float diff2 = max(dot(Normal, LightDir2), 0.0);


  if (material.texture == 0) {  
    FragColor = material.ambient + material.diffuse * (diff1 + diff2);
  } else {
    vec4 texture_color = texture(texture_diffuse1, TexCoord);
    // vec4 texture_color = vec4(1.0, 0.0, 0.0, 1.0);
    FragColor = texture_color * (material.ambient + diff1 + diff2);
  }
  

//   vec3 LightDir2 = vec3(-LightDir.x, -LightDir.y, LightDir.z);

  

//   vec4 amb = vec4(0.1, 0.1, 0.1, 1.0);

//   FragColor = texture(texture_diffuse1, TexCoord);
  
  

  // FragColor = ourColor;
  // FragColor = vec4(1.0f);
}
