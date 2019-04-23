#version 460 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 boneweights;

uniform mat4 model;
uniform mat4 toerotMatrix;
uniform mat4 legrotMatrix;
uniform mat4 projection;
uniform mat4 m2btoe;
uniform mat4 m2bleg;
uniform mat4 b2mtoe;
uniform mat4 b2mleg;

void main()
{
	mat4 bonetransform = boneweights.z*b2mtoe*toerotMatrix*m2btoe;
	bonetransform = bonetransform + boneweights.w*b2mleg*legrotMatrix*m2bleg;
	bonetransform = bonetransform + (boneweights.x + boneweights.y)*mat4(1.0);
	gl_Position = projection*model*bonetransform*vec4(aPos, 1.0);
}
