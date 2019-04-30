#version 460 core

layout(location = 0) in vec4 aPos;
layout(location = 1) in vec4 boneweights;
layout(location = 2) in float aOffset;
layout(location = 3) in mat4 instanceMatrix;
layout(location = 7) in mat4 toerotMatrix;
layout(location = 11) in mat4 legrotMatrix;

uniform mat4 u_P;
uniform vec3 bbox;
uniform mat4 m2btoe;
uniform mat4 m2bleg;
uniform mat4 b2mtoe;
uniform mat4 b2mleg;

flat out int instanceid;

void main()
{
	instanceid = gl_InstanceID;
	mat4 bonetransform = boneweights.z*b2mtoe*toerotMatrix*m2btoe;
	bonetransform = bonetransform + boneweights.w*b2mleg*legrotMatrix*m2bleg;
	bonetransform = bonetransform + (boneweights.x + boneweights.y)*mat4(1.0);
	//vec4 pos = u_P * instanceMatrix *bonetransform*aPos;
	vec4 pos = u_P * instanceMatrix * bonetransform * aPos;

	pos = pos / pos.w;
	float xPos = (pos.x + 1.0) * 640.0;
	float yPos = (pos.y + 1.0) * 360.0;
	// box coords between 0 and 1
	xPos = (xPos - float(bbox.x)) / float(bbox.z);
	yPos = (yPos - float(bbox.y)) / float(bbox.z);
	// get correct pos
	xPos = 2.0 / 256.0 * xPos - 1.0;
	xPos = xPos + aOffset;
	yPos = 2.0 * yPos - 1.0;

	gl_Position = vec4(xPos, yPos, pos.z, pos.w);
}
