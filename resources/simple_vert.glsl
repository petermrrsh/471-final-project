#version  330 core
layout(location = 0) in vec4 vertPos;
layout(location = 1) in vec3 vertNor;
layout(location = 2) in vec3 vertTex;
layout(location = 3) in vec3 instancePos;
uniform mat4 P;
uniform mat4 V;
uniform mat4 M;
uniform vec3 lightPos;

out vec3 fragNor;
out vec3 lightDir;
out vec3 EPos;

void main()
{
	

	vec4 pos = vec4(instancePos, 1.0f) + (M * vertPos);

	
	gl_Position = P * V * pos;
	
	fragNor = (V*M * vec4(vertNor, 0.0)).xyz;
	lightDir = vec3(V*(vec4(lightPos - (M*vertPos).xyz, 0.0)));

	EPos = vec3(V * M * vertPos);
}
