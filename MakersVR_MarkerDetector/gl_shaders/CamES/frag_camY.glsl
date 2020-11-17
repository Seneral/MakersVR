#version 100
#extension GL_OES_EGL_image_external : require

precision mediump float;

uniform samplerExternalOES imageY;

uniform int width;
uniform int height;

varying vec2 uv;

void main()
{
    float value = texture2D(imageY, uv).r;
    vec3 color = vec3(value, value, value);
	
    gl_FragColor = vec4(color.rgb, 1.0);
}
