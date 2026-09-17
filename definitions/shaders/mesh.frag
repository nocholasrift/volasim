#version 330 core
in vec3 Normal;
in vec3 FragPos;
uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 color;
out vec4 FragColor;
void main() {
  vec3 norm = Normal;
  vec3 lightDir = normalize(lightPos - FragPos);
  float diff = max(dot(norm, lightDir), 0.0);
  vec3 diffuse = diff * lightColor;
  float ambient = 0.1;
  vec3 result = (diffuse + ambient) * color;
  FragColor = vec4(result, 1.0);
}
