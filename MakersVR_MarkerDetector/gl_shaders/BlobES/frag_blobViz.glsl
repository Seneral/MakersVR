#version 100

precision mediump float;
precision lowp int;

uniform sampler2D image;

uniform int width;
uniform int height;

// View Bounds
uniform int minX, minY, maxX, maxY;

varying vec2 uv;

float grayscale(vec2 uvCoord)
{
    vec3 color = vec3(texture2D(image, uvCoord));
    return (color.r + color.g + color.b) / 3.0;
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
    vec2 t = vec2(
        (uv.x*float(maxX-minX) + float(minX)) / float(width), 
        (uv.y*float(maxY-minY) + float(minY)) / float(height));

    vec4 tex = texture2D(image, t);
    vec3 color = tex.rgb;
    float grsc = (color.r+color.g+color.b)/3.0;
    float value = tex.a;
    
    float maxPlus = maxVal(t + 1.0*dX, t - 1.0*dX, t + 1.0*dY, t - 1.0*dY);
    float maxCross = maxVal(t + 1.0*dX + 1.0*dY, t - 1.0*dX + 1.0*dY, t - 1.0*dX - 1.0*dY, t + 1.0*dX - 1.0*dY);
    float maxL = maxVal(t - 2.0*dX + 1.0*dY, t - 2.0*dX + 0.0*dY, t - 2.0*dX - 1.0*dY, t - 2.0*dX - 2.0*dY);
    float maxT = maxVal(t - 1.0*dX - 2.0*dY, t - 0.0*dX - 2.0*dY, t + 1.0*dX - 2.0*dY, t + 2.0*dX - 2.0*dY);
    float maxR = maxVal(t + 2.0*dX - 1.0*dY, t + 2.0*dX - 0.0*dY, t + 2.0*dX + 1.0*dY, t + 2.0*dX + 2.0*dY);
    float maxB = maxVal(t + 1.0*dX + 2.0*dY, t + 0.0*dX + 2.0*dY, t - 1.0*dX + 2.0*dY, t - 2.0*dX + 2.0*dY);
    float maxOuter = max(maxL, max(maxT, max(maxR, maxB)));
    float maxInner = max(maxPlus, maxCross);
    
    //float pointWeight = min(1.0, value + maxPlus*3.0/2.0 + maxCross*3.0/3.0 + maxOuter*3.0/5.0);
    //gl_FragColor = vec4(mix(mix(color, vec3(0.0,1.0,0.0), pointWeight), vec3(1.0,0.0,0.0), value), 1.0);
    //gl_FragColor = vec4(value, maxInner-value, 0.0, 1.0);
//	gl_FragColor = vec4(color, 1.0);
    //gl_FragColor = vec4(mix(vec3(grsc,grsc,grsc), vec3(1.0,0.0,0.0), value), 1.0);
	gl_FragColor = vec4(mix(color, vec3(0.0,1.0,0.0), value), 1.0);
    //gl_FragColor = vec4(grsc,grsc,grsc, 1.0);
//    gl_FragColor = vec4(value,value,value, 1.0);
//    gl_FragColor = vec4(color, 1.0);
//	if (float(width) * uv.x)
//	float diffX = abs(((float(width)*uv.x) - float(width/4)*4.0 * uv.x));
//	float diffY = abs(((float(height)*uv.y) - float(height/8)*8.0 * uv.y));
//	float border = testLE(diffX, 0.000000005) + testLE(diffY, 0.00000001);
//    gl_FragColor = vec4(mix(color, vec3(1.0,0.0,0.0), border), 1.0);
	
}
