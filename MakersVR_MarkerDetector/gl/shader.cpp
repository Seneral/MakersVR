#include "shader.hpp"

#include "defines.hpp"

#include <iostream>
#include <fstream>
#include <cstring>

/*
 * Read string file from disk
 */
std::string readFile(const char *filePath)
{
	std::ifstream fs(filePath, std::ios::in | std::ios::binary);
	std::string contents;
	if (fs.is_open())
	{
		fs.seekg(0, std::ios::end);
		contents.resize(fs.tellg());
		fs.seekg(0, std::ios::beg);
		fs.read(&contents[0], contents.size());
		fs.close();
	}
	else
	{
		std::cerr << "Could not open " << filePath << ": " << strerror(errno) << "!" << std::endl;
	}
	return contents;
}

/*
 * Load shader text from file and compile
 */
GLuint loadShader (const char* fileName, int type)
{
	std::string shaderSrcStr = readFile(fileName);
	const char* shaderSrc = shaderSrcStr.c_str();
	GLuint shader = glCreateShader(type);
	if (shader == 0)
	{
		std::cerr << "Failed to create shader for '" << fileName << "'! Check context!" << std::endl;
		return 0;
	}
	glShaderSource(shader, 1, &shaderSrc, NULL);
	glCompileShader(shader);
	int success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		char infoLog[512];
		glGetShaderInfoLog(shader, 512, NULL, infoLog);
		std::cerr << "Failed to compile shader '" << fileName << "'! \n" << infoLog;
		std::cerr << shaderSrc;
		return 0;
	}
	return shader;
}

ShaderProgram::ShaderProgram(const char* vertShaderFile, const char* fragShaderFile)
{
	ID = 0;
	GLuint vertShader = loadShader(vertShaderFile, GL_VERTEX_SHADER);
	GLuint fragShader = loadShader(fragShaderFile, GL_FRAGMENT_SHADER);
	if (vertShader != 0 && fragShader != 0)
	{
		ID = glCreateProgram();
		glAttachShader(ID, vertShader);
		glAttachShader(ID, fragShader);
		// Vertex data, bound to fixed channels
		glBindAttribLocation(ID, vPosAdr, "vPos");
		glBindAttribLocation(ID, vColAdr, "vCol");
		glBindAttribLocation(ID, vUVAdr, "vTex");
		glBindAttribLocation(ID, vNrmAdr, "vNrm");
		// Link
		glLinkProgram(ID);
		int success;
		glGetProgramiv(ID, GL_LINK_STATUS, &success);
		if (!success)
		{
			char infoLog[512];
			glGetProgramInfoLog(ID, 512, NULL, infoLog);
			std::cerr << "Failed to link shader program for vert '" << vertShaderFile << "' + frag '"  << fragShaderFile << "'! \n" << infoLog << "\n";
			ID = 0;
		}
		else
		{
			uImageAdr = glGetUniformLocation(ID, "image");
			uWidthAdr = glGetUniformLocation(ID, "width");
			uHeightAdr = glGetUniformLocation(ID, "height");
		}
	}
	if (vertShader != 0) glDeleteShader(vertShader);
	if (fragShader != 0) glDeleteShader(fragShader);
}

ShaderProgram::~ShaderProgram ()
{
	glDeleteProgram(ID);
}

void ShaderProgram::use (void)
{
	glUseProgram(ID);
}
