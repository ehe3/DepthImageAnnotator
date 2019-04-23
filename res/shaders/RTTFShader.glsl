#version 460 core

precision mediump float;

//zNear = 0.1f;
//zFar = 3.0f;
//instances = 128;

flat in int instanceid;

out vec4 FragColor;

void main()
{
	int lowerBound = 64*instanceid;
	int upperBound = lowerBound+64;
	if ((gl_FragCoord.x <	lowerBound) || (gl_FragCoord.x > upperBound)) {
		discard;
	}
	float zTrans = 2.0 * gl_FragCoord.z - 1.0;
	float nonNormalFragDepth = 0.6 / (3.1 - 2.9 * zTrans);
	gl_FragDepth = nonNormalFragDepth / 3.0;
}

