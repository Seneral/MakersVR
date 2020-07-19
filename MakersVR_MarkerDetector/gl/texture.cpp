#include "defines.hpp"
#include "texture.hpp"

#include <iostream>

static int nextPOT (int NPOT)
{
	NPOT--;
	NPOT |= NPOT >> 1;
	NPOT |= NPOT >> 2;
	NPOT |= NPOT >> 4;
	NPOT |= NPOT >> 8;
	NPOT++;
	return std::max(64, NPOT);
}

ExternalTexture::ExternalTexture (GLint texHandle, int Width, int Height)
{
	ID = texHandle;
	width = Width;
	height = Height;
}
void ExternalTexture::setSource (ShaderProgram *shader, int slot)
{
	glUniform1i(shader->uImageAdr, slot);
	glUniform1i(shader->uWidthAdr, width);
	glUniform1i(shader->uHeightAdr, height);
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_EXTERNAL_OES, ID);
}

FrameRenderTarget::FrameRenderTarget (int Width, int Height, GLenum format, GLenum type)
{
	width = Width;
	height = Height;
	// Framebuffer
	glGenFramebuffers(1, &FBO_ID);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO_ID);
	// Color Buffer
	glGenTextures(1, &colorBuffer_ID);
	glBindTexture(GL_TEXTURE_2D, colorBuffer_ID);
	glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, type, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); // GL_LINEAR / GL_NEAREST
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // GL_LINEAR / GL_NEAREST
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorBuffer_ID, 0);
	// Check
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		std::cout << "Error: Framebuffer not complete: " << glCheckFramebufferStatus(GL_FRAMEBUFFER) << "\n";
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
FrameRenderTarget::~FrameRenderTarget (void)
{
	glDeleteTextures(1, &colorBuffer_ID);
	glDeleteFramebuffers(1, &FBO_ID);
}
void FrameRenderTarget::setTarget (void)
{
	glViewport(0, 0, width, height);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO_ID);
}
void FrameRenderTarget::setSource (ShaderProgram *shader, int slot)
{
	glUniform1i(shader->uWidthAdr, width);
	glUniform1i(shader->uHeightAdr, height);
	glUniform1i(shader->uImageAdr, slot);
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, colorBuffer_ID);
}

BufferRenderTarget::BufferRenderTarget (int Width, int Height, GLenum format)
{
	width = Width;
	height = Height;
	// Framebuffer
	glGenFramebuffers(1, &FBO_ID);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO_ID);
	// Color Buffer
	glGenRenderbuffers(1, &colorBuffer_ID);
	glBindRenderbuffer(GL_RENDERBUFFER, colorBuffer_ID);
	glRenderbufferStorage(GL_RENDERBUFFER, format, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBuffer_ID);
	// Check
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		std::cout << "Error: Framebuffer not complete: " << glCheckFramebufferStatus(GL_FRAMEBUFFER) << "\n";
		std::cout << "Width " << width << " height"  << height << "\n";
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
BufferRenderTarget::~BufferRenderTarget (void)
{
	glDeleteTextures(1, &colorBuffer_ID);
	glDeleteFramebuffers(1, &FBO_ID);
}
void BufferRenderTarget::setTarget (void)
{
	glViewport(0, 0, width, height);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO_ID);
}
void BufferRenderTarget::setSource (ShaderProgram *shader, int slot)
{
	glUniform1i(shader->uWidthAdr, width);
	glUniform1i(shader->uHeightAdr, height);
	glUniform1i(shader->uImageAdr, slot);
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, colorBuffer_ID);
}

VCSMRenderTarget::VCSMRenderTarget (int Width, int Height, EGLDisplay Display)
{
	width = Width;
	height = Height;
	bufferWidth = nextPOT(width);
	bufferHeight = nextPOT(height);
	std::cout << "Texture Size " << width << "x" << height << std::endl;
	std::cout << "Buffer Size " << bufferWidth << "x" << bufferHeight << std::endl;
	eglDisplay = Display;
	// Framebuffer
	glGenFramebuffers(1, &FBO_ID);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO_ID);
	// Create texture handle
	glGenTextures(1, &TEX_ID);
	glBindTexture(GL_TEXTURE_2D, TEX_ID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); // GL_LINEAR / GL_NEAREST
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // GL_LINEAR / GL_NEAREST
	// Allocate VCOS Shared Memory and assign to texture
	VCSM.width = bufferWidth;
    VCSM.height = bufferHeight;
    eglImg = eglCreateImageKHR(eglDisplay, EGL_NO_CONTEXT, EGL_IMAGE_BRCM_VCSM, &VCSM, NULL);
	CHECK_GL();
    glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, eglImg);

	VCSM_CACHE_TYPE_T cacheType;
	vcsmBuffer = (uint8_t*)vcsm_lock_cache(VCSM.vcsm_handle, VCSM_CACHE_TYPE_HOST, &cacheType);
	memset(vcsmBuffer, 0, bufferWidth * bufferHeight * 2);
	vcsm_unlock_ptr(vcsmBuffer);

	CHECK_GL();
	// Bind texture to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, TEX_ID, 0);
	if (eglImg == EGL_NO_IMAGE_KHR || VCSM.vcsm_handle == 0)
    {
        std::cout << VCOS_FUNCTION << ": Failed to create EGL VCSM image\n";
        return;
	}
	CHECK_GL();
	// Check
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		std::cout << "Error: Framebuffer not complete: " << glCheckFramebufferStatus(GL_FRAMEBUFFER) << "\n";
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
VCSMRenderTarget::~VCSMRenderTarget (void)
{
	glDeleteTextures(1, &TEX_ID);
	glDeleteFramebuffers(1, &FBO_ID);
	eglDestroyImageKHR(eglDisplay, eglImg);
}
void VCSMRenderTarget::setTarget (void)
{
	glViewport(0, 0, width, height);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO_ID);
}
void VCSMRenderTarget::setSource (ShaderProgram *shader, int slot)
{
	glUniform1i(shader->uWidthAdr, bufferWidth);
	glUniform1i(shader->uHeightAdr, bufferHeight);
	glUniform1i(shader->uImageAdr, slot);
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_2D, TEX_ID);
}
uint8_t* VCSMRenderTarget::lock (void)
{
	VCSM_CACHE_TYPE_T cacheType;
	vcsmBuffer = (uint8_t*)vcsm_lock_cache(VCSM.vcsm_handle, VCSM_CACHE_TYPE_HOST, &cacheType);
    if (!vcsmBuffer) std::cout << "Failed to lock VCSM buffer!\n";
    return vcsmBuffer;
}
void VCSMRenderTarget::unlock (void)
{
	vcsm_unlock_ptr(vcsmBuffer);
}
