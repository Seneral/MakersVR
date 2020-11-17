#version 100
#extension GL_OES_EGL_image_external : require

precision mediump float;

uniform samplerExternalOES imageY;
uniform samplerExternalOES imageU;
uniform samplerExternalOES imageV;

uniform int width;
uniform int height;

varying vec2 uv;

void main()
{
    float value = texture2D(imageY, uv).r;
	float y = 1.1643 * (value - 0.0625);
	float u = texture2D(imageU, uv).r - 0.5;
	float v = texture2D(imageV, uv).r - 0.5;
    vec3 color = vec3(y + 1.5958*v, y - 0.39173*u - 0.81290*v, y + 2.017*u);
    
    gl_FragColor = vec4(color.rgb, 1.0);
}
