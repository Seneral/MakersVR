#ifndef EGLUTIL_H
#define EGLUTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include "bcm_host.h"

#include "applog.h"

#define CHECK_EVAL(EVAL, MSG, ERRHANDLER) \
	if (!(EVAL)) { \
		vcos_log_error(MSG); \
		goto ERRHANDLER; \
	}

typedef struct EGL_Setup
{
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	int versionMinor;
	int versionMajor;
} EGL_Setup;

int setupEGL(EGL_Setup *setup, EGLNativeWindowType window);

void terminateEGL(EGL_Setup *setup);

/* Create native window (basically just a rectangle on the screen we can render to */
int createNativeWindow(EGL_DISPMANX_WINDOW_T *window);

#ifdef __cplusplus
}
#endif

#endif
