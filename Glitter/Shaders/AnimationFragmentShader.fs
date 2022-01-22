#version 330 core
out vec4 FragColor;

struct Material{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

struct Light {
	// Direction has to be in world coordinates.
	vec3 direction;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

uniform Light light;
uniform Material material;
uniform vec3 viewPos;

in vec3 FragPos;
in vec3 wNormal;

void main()
{
	//ambient
	vec3 ambient = light.ambient * material.ambient;
	
	//diffuse
	vec3 norm = normalize(wNormal);
	vec3 lightDir = normalize(-light.direction);
	float diff = max(dot(norm, lightDir), 0.0);
	vec3 diffuse = light.diffuse * diff * material.diffuse;

	//specular
	vec3 viewDir = normalize(viewPos - FragPos);
	vec3 reflectDir = reflect(-lightDir, norm);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
	vec3 specular = light.specular * spec * material.specular;

	vec3 result = ambient + diffuse + specular;
	FragColor = vec4(result, 1.0);
}