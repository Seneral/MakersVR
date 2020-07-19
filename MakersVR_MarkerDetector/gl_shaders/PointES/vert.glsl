#version 100

attribute vec3 pos;

varying vec2 uv;

void main()
{
    gl_Position = vec4(pos.xy, 0.5, 1.0);
	gl_PointSize = max(1.0, pos.z * 1.0);
}
