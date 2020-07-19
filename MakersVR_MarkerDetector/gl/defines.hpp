
#ifndef DEF_DEFINES
#define DEF_DEFINES

#include <GLES2/gl2.h>
#include <iostream>

static const float PI = 3.14159265358979323846;

// Predefined shader location of vertex attributes
static const GLint vPosAdr = 0, vColAdr = 1, vUVAdr = 2, vNrmAdr = 3;

#define CHECK_GL() checkGL(__FILE__, __LINE__)

static void checkGL(const char* file, uint32_t line)
{
	GLenum error = glGetError();
	while (error != GL_NO_ERROR)
	{
		std::cerr << "GL error: file " << file << " line "<< line << " error " << error << std::endl;
		error = glGetError();
	}
}

#endif
