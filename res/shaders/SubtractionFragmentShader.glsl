#version 460 core

precision mediump float;

out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;
uniform sampler2D gendepTexture;

float threshold = 0.80;

void main()
{
	vec4 ref = texture(screenTexture, TexCoords);
	vec4 rend = texture(gendepTexture, TexCoords);
	gl_FragDepth = min(3*abs(ref.z - rend.z), threshold);
	//gl_FragDepth = abs(ref.z - rend.z);
}
