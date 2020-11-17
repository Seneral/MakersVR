#ifndef EGLUTIL_H
#define EGLUTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include "bcm_host.h"

typedef struct EGL_Setup
{
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	int versionMinor;
	int versionMajor;
} EGL_Setup;

typedef struct EGL_Window
{
	EGL_DISPMANX_WINDOW_T eglWindow;
	DISPMANX_DISPLAY_HANDLE_T display;
} EGL_Window;

/*
 * Create EGL setup rendering to pixelbuffer
 */
int setupEGLPBuffer(EGL_Setup *setup);

/*
 * Create EGL setup rendering to window on screen
 ' If sharedSetup is provided, both setups will share their context
 */
int setupEGLWindow(EGL_Setup *setup, EGL_Window *window, EGL_Setup *sharedSetup);

/*
 * Cleanup EGL setup
 */
void terminateEGL(EGL_Setup *setup);

/*
 * Create native window with EGL element
 */
int createWindow(EGL_Window *window);

/*
 * Destroy native window with EGL element
 */
int destroyWindow(EGL_Window *window);

#ifdef __cplusplus
}
#endif

#endif
