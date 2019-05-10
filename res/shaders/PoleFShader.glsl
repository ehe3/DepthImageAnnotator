#version 460 core

precision mediump float;

//zNear = 0.1f;
//zFar = 3.0f;
//instances = 128;

flat in int instanceid;

out vec4 FragColor;

void main()
{
	int lowerBound = 32*instanceid;
	int upperBound = lowerBound+32;
	if ((gl_FragCoord.x <	lowerBound) || (gl_FragCoord.x > upperBound)) {
		discard;
	}
	gl_FragDepth = 1.0;
}

