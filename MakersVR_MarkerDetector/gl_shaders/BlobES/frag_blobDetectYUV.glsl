#version 100
#extension GL_OES_EGL_image_external : require

precision mediump float;

uniform samplerExternalOES imageY;
uniform samplerExternalOES imageU;
uniform samplerExternalOES imageV;

uniform int width;
uniform int height;

varying vec2 uv;


float grayscale(vec2 uvCoord)
{
    return texture2D(imageY, uvCoord).r;
}
float maxVal(vec2 uv1, vec2 uv2, vec2 uv3, vec2 uv4) 
{
    return max(grayscale(uv1), max(grayscale(uv2), max(grayscale(uv3), grayscale(uv4))));
}
float testLE(float value, float target) 
{
    return min(1.0, max(0.0, min(1.0, (target-value)*1000.0)) * 100000.0);
}

void main()
{
    vec2 dX = vec2(1.0/float(width), 0.0);
    vec2 dY = vec2(0.0, 1.0/float(height));

    float value = texture2D(imageY, uv).r;
	float y = 1.1643 * (value - 0.0625);
	float u = texture2D(imageU, uv).r - 0.5;
	float v = texture2D(imageV, uv).r - 0.5;
    vec3 color = vec3(y + 1.5958*v, y - 0.39173*u - 0.81290*v, y + 2.017*u);
    
    float maxPlus = maxVal(uv + 1.0*dX, uv - 1.0*dX, uv + 1.0*dY, uv - 1.0*dY);
    float maxCross = maxVal(uv + 1.0*dX + 1.0*dY, uv - 1.0*dX + 1.0*dY, uv - 1.0*dX - 1.0*dY, uv + 1.0*dX - 1.0*dY);
    float maxL = maxVal(uv - 2.0*dX + 1.0*dY, uv - 2.0*dX + 0.0*dY, uv - 2.0*dX - 1.0*dY, uv - 2.0*dX - 2.0*dY);
    float maxT = maxVal(uv - 1.0*dX - 2.0*dY, uv - 0.0*dX - 2.0*dY, uv + 1.0*dX - 2.0*dY, uv + 2.0*dX - 2.0*dY);
    float maxR = maxVal(uv + 2.0*dX - 1.0*dY, uv + 2.0*dX - 0.0*dY, uv + 2.0*dX + 1.0*dY, uv + 2.0*dX + 2.0*dY);
    float maxB = maxVal(uv + 1.0*dX + 2.0*dY, uv + 0.0*dX + 2.0*dY, uv - 1.0*dX + 2.0*dY, uv - 2.0*dX + 2.0*dY);
    float maxOuter = max(maxL, max(maxT, max(maxR, maxB)));
    float maxInner = max(maxPlus, maxCross);
    
    //float isPoint = testLE(maxOuter, value) * testLE(maxInner, value) * testLE(0.2, value);
    //float isPoint = testLE(maxOuter, value*1.5) * testLE(maxInner, value*2.0) * testLE(0.25, value);
    float isPoint = testLE(maxOuter, value*2.0) * testLE(maxInner, value*1.5) * testLE(0.2, value);
    //float isPoint = testLE(maxOuter, value*1.00001) * testLE(maxInner, value*1.000001) * testLE(0.25, value);
    
    gl_FragColor = vec4(color.rgb, isPoint);
}
