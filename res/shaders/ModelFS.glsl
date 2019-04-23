#version 460 core

precision mediump float;

void main()
{
	float zTrans = 2.0 * gl_FragCoord.z - 1.0;
	float nonNormalFragDepth = 0.6 / (3.1 - 2.9 * zTrans);
	gl_FragDepth = nonNormalFragDepth / 3.0;
}
