#version 100

attribute vec3 vPos;
attribute vec2 vTex;

varying vec2 uv;

void main()
{
    gl_Position = vec4(vPos.xyz, 1.0);
    uv = vTex;
}
