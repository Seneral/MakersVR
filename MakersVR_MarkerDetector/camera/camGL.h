#ifndef CAMGL_H
#define CAMGL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <GLES2/gl2.h>
#include "eglUtil.h"

#define CAMGL_SUCCESS			0
#define CAMGL_QUIT				1
#define CAMGL_ERROR				2
#define CAMGL_ALREADY_STARTED	3
#define CAMGL_START_FAILED		4
#define CAMGL_NOT_STARTED		5
#define CAMGL_GL_ERROR			6
#define CAMGL_NO_FRAMES			7

typedef enum CamGL_FrameFormat
{
	CAMGL_RGB,
	CAMGL_Y,
	CAMGL_YUV
} CamGL_FrameFormat;

typedef struct CamGL_Frame
{
	CamGL_FrameFormat format;
	uint16_t width;
	uint16_t height;
	GLuint textureRGB;
	GLuint textureY;
	GLuint textureU;
	GLuint textureV;
} CamGL_Frame;

typedef struct CamGL_Params
{
	CamGL_FrameFormat format;
	uint16_t width;
	uint16_t height;
	uint16_t fps;
	uint32_t shutterSpeed;
	int32_t iso;
} CamGL_Params;

typedef struct CamGL CamGL;

CamGL *camGL_create(EGL_Setup setup, const CamGL_Params *params);
void camGL_destroy(CamGL *camGL);

/** Start camera */
uint8_t camGL_startCamera(CamGL *camGL);

/** Explicitly stop gl camera stream */
uint8_t camGL_stopCamera(CamGL *camGL);

/* Returns constant reference to frame  */
CamGL_Frame* camGL_getFrame(CamGL *camGL);

/* Returns whether there is a new camera frame available */
uint8_t camGL_hasNextFrame(CamGL *camGL);

/* Updates the frame structure with the most recent camera frame. 
 * If no camera frame is available yet, blocks until there is.
 * If the last frame has not been returned yet or camera stream was interrupted, returns error code. */
int camGL_nextFrame(CamGL *camGL);

#ifdef __cplusplus
}
#endif

#endif
