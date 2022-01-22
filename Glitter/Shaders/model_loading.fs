#version 330 core
out vec4 FragColor;

uniform vec3 lightDirection;

in vec3 worldNormal;

void main()
{
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    vec3 ambient = vec3(0.1) * vec3(0.4, 0.4, 0.4);
    vec3 norm = normalize(worldNormal);
    float diff = max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = vec3(0.2) * (diff * vec3(0.4, 0.4, 0.4));
    FragColor = vec4(ambient + diffuse, 1.0);
}
