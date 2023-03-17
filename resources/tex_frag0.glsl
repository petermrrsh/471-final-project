#version 330 core
uniform sampler2D Texture0;
uniform int flip;

in vec3 fragNor;
in vec3 lightDir;

in vec2 vTexCoord;
out vec4 Outcolor;

void main() {
  vec4 texColor0 = texture(Texture0, vTexCoord);

  vec3 normal = normalize(fragNor);
  if (flip == 1) {
		normal = normal * -1.0f;
	}
	
	vec3 light = normalize(lightDir);

  //to set the out color as the texture color - uncomment later on
  float dC = max(0, dot(normal, light));
  Outcolor = texColor0;
  
  //to set the outcolor as the texture coordinate (for debugging)
	//Outcolor = vec4(vTexCoord.s, vTexCoord.t, 0, 1);
}

