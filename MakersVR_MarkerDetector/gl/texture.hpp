#ifndef DEF_TEXTURE
#define DEF_TEXTURE

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <interface/vcsm/user-vcsm.h>

#include "shader.hpp"


/*
 * Abstract GL Texture
 */
class Texture
{
	public:
	int width, height;

	virtual void setSource (ShaderProgram *shader, int slot) = 0;
};

/*
 * External EGL Texture
 */
class ExternalTexture : public Texture
{
	public:
	ExternalTexture (GLint texHandle, int Width, int Height);
	virtual void setSource (ShaderProgram *shader, int slot) override;

	protected:
	GLuint ID;
};

/*
 * Abstract Render Target
 */
class RenderTarget : public Texture
{
	public:
	virtual void setTarget (void) = 0;
	virtual void setSource (ShaderProgram *shader, int slot) = 0;

	protected:
	GLuint FBO_ID;
};

/*
 * A simple texture target with one color buffer (read-write)
 */
class FrameRenderTarget : public RenderTarget
{
	public:
	FrameRenderTarget (int Width, int Height, GLenum format, GLenum type);
	~FrameRenderTarget (void);
	void setTarget (void) override;
	void setSource (ShaderProgram *shader, int slot) override;

	private:
	GLuint colorBuffer_ID;
};

/*
 * A simple RenderBuffer target with one color buffer (write-only)
 */
class BufferRenderTarget : public RenderTarget
{
	public:
	BufferRenderTarget (int Width, int Height, GLenum format);
	~BufferRenderTarget (void);
	void setTarget (void) override;
	void setSource (ShaderProgram *shader, int slot) override;

	private:
	GLuint colorBuffer_ID;
};

/*
 * A VCOS shared memory render target with one RGBA buffer (read-write)
 * VCOS only support POT textures, so only a part will be used
 */
class VCSMRenderTarget : public RenderTarget
{
	public:
	int bufferWidth, bufferHeight;
	VCSMRenderTarget (int Width, int Height, EGLDisplay Display);
	~VCSMRenderTarget (void);
	void setTarget (void) override;
	void setSource (ShaderProgram *shader, int slot) override;
	uint8_t* lock(void);
	void unlock(void);

	private:
	GLuint TEX_ID;
	struct egl_image_brcm_vcsm_info VCSM;
	EGLImageKHR eglImg;
	uint8_t *vcsmBuffer;
	EGLDisplay eglDisplay;
};

#endif
