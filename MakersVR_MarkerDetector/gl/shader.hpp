#ifndef DEF_SHADER
#define DEF_SHADER

#include <GLES2/gl2.h>

#include <string>

/*
 * Read string file from disk
 */
std::string readFile(const char *filePath);

/*
 * Load shader text from file and compile
 */
GLuint loadShader (const char* fileName, int type);

/*
 * A compiled and linked shader program with vertex and fragment shaders
 */
class ShaderProgram
{
	public:
	GLuint ID;
	GLint uImageAdr = 0, uWidthAdr = 1, uHeightAdr = 2;

	ShaderProgram(const char* vertShaderFile, const char* fragShaderFile);
	~ShaderProgram ();
	void use (void);
};

#endif
