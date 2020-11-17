#version 100
#extension GL_OES_EGL_image_external : require

precision mediump float;

uniform samplerExternalOES imageRGB;

uniform int width;
uniform int height;

varying vec2 uv;

void main()
{
    vec3 color = vec3(texture2D(imageRGB, uv));

    gl_FragColor = vec4(color.rgb, 1.0);
}
