#version 460 core

precision mediump float;

out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;
uniform sampler2D gendepTexture;

float threshold = 1.0;
float rotationFactor = 1.0;

void main()
{
	vec4 ref = texture(screenTexture, TexCoords);
	vec4 rend = texture(gendepTexture, TexCoords);
	if (ref.z == 1.0 && rend.z != 1.0)
	{
		gl_FragDepth = rend.z; 
	}
	else if (ref.z != 1.0 && rend.z == 1.0)
	{
		gl_FragDepth = 1.0;
	}
	else
	{
		gl_FragDepth = 0;
		gl_FragDepth = min(rotationFactor*abs(ref.z - rend.z), threshold);
		//gl_FragDepth = min(threshold, threshold);
		//gl_FragDepth = abs(ref.z - rend.z);
	}
	//gl_FragDepth = abs(ref.z - rend.z);
}
