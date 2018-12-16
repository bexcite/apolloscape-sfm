#version 330 core


struct Material {

    // Contribution levels
    vec4 diffuse;
    vec4 ambient;

    // Flags of texture presents
    int texture_diffuse;
    int texture_ambient;
    int texture_specular;

    sampler2D texture_ambient1;
    sampler2D texture_diffuse1;
};

out vec4 FragColor;

in vec2 TexCoord;
in vec3 FragPos;
in vec3 Normal;

// TODO: Change this to an array
in vec3 LightDir1;
in vec3 LightDir2;

uniform Material material;

void main() {

  float diff1 = max(dot(Normal, LightDir1), 0.0);
  float diff2 = max(dot(Normal, LightDir2), 0.0);

  vec4 result = vec4(0.0);

  vec4 ambient = material.ambient;

  vec4 diffuse = material.diffuse * (diff1 + diff2);;

  // ======= Diffuse pa rt ===============
  if (material.texture_diffuse > 0) {
    vec4 texture_color = texture(material.texture_diffuse1, TexCoord);
    diffuse = texture_color * (diff1 + diff2);
    ambient = texture_color * ambient;
  } 
  // else {
  //   diffuse = material.diffuse * (diff1 + diff2);
  // }
  
  // ======= Ambient part ===============
  if (material.texture_ambient > 0) {
    vec4 texture_color = texture(material.texture_ambient1, TexCoord);
    float a = ambient.a;
    ambient = texture_color;
    ambient.a = a;
  } 

  FragColor = ambient + diffuse;
  // FragColor.a = 0.8;
    

}
