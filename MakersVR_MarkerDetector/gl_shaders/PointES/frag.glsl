#version 100

precision mediump float;

float testLE(float value, float target) 
{
    return min(1.0, min(1.0, max(0.0, min(1.0, (target-value)*100.0)) * 100000.0) * 100000.0);
}

void main()
{
    //vec2 center = uv-vec2(0.5, 0.5);
    //float inCircle = testLE(center.x*center.x + center.y*center.y, 1.0);
    gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
