#version 100

precision highp float;
precision highp int;

uniform sampler2D image;

uniform int width;
uniform int height;

varying vec2 uv;

// Expects: Blobiness in image alpha (1 or 0)
// Encodes binary mask in 8 bits per component for a 8x4 region (kernel)
void main()
{
    vec2 dX = vec2(1.0/float(width), 0.0);
    vec2 dY = vec2(0.0, 1.0/float(height));
    vec2 uvS = uv - 1.5*dX - 1.5*dY - 2.0*dX;

    float components[4];    
    for (int c = 0; c < 4; c++) 
    {
		int component = 0;
        for (int x = 1; x >= 0; x--) 
        {
            for (int y = 0; y < 4; y++) 
            {
                component = component * 2 + int(texture2D(image, uvS + float(c*2+x)*dX + float(y)*dY).a);
            }
        }
        // Encode float[0-1] <=> 8bit integer[0-255]
        // Components of RGBA4 are 8bit normalized fixed-points (see OpenGL ES 3.2 full spec)
        components[c] = float(component) / 255.0;

    }

    gl_FragColor = vec4(components[0], components[1], components[2], components[3]);
}
