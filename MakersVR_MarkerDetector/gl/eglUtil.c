#include "eglUtil.h"

#include <stdio.h>

#define CHECK_EVAL(EVAL, ERRHANDLER, ...) \
if (!(EVAL)) { \
	fprintf(stderr, __VA_ARGS__); \
	goto ERRHANDLER; \
}

int setupEGLPBuffer(EGL_Setup *setup)
{
	EGLBoolean estatus;
	GLenum glerror;
	const EGLint configAttributes[] =
	{
		EGL_CONFORMANT,		EGL_OPENGL_ES2_BIT,
		EGL_SURFACE_TYPE,	EGL_PBUFFER_BIT,
		EGL_RED_SIZE,		5,
		EGL_GREEN_SIZE,		6,
		EGL_BLUE_SIZE,		5,
		EGL_ALPHA_SIZE,		0,
		EGL_DEPTH_SIZE,		0,
		EGL_NONE
	};
	const EGLint contextAttributes[] =
	{
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};

	CHECK_EVAL((glerror = glGetError()) == GL_NO_ERROR, error_display, "GL error before init: error 0x%04x\n", glerror);

	// Get display
	setup->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	CHECK_EVAL(setup->display != EGL_NO_DISPLAY, error_display, "Failed to get EGL display!\n");
	estatus = eglInitialize(setup->display, &setup->versionMajor, &setup->versionMinor);
	CHECK_EVAL(estatus, error_surface, "Failed to initialized EGL!\n");

	// Choose config according to attributes
	EGLConfig config;
	EGLint numConfigs;
	estatus = eglChooseConfig(setup->display, configAttributes, &config, 1, &numConfigs);
	CHECK_EVAL(estatus, error_surface, "Failed to choose config!\n");

	// Bind OpenGL ES API (Needed?)
	estatus = eglBindAPI(EGL_OPENGL_ES_API);
	CHECK_EVAL(estatus, error_surface, "Failed to bind OpenGL ES API!\n");

	// Create surface
	setup->surface = eglCreatePbufferSurface(setup->display, config, NULL);
	CHECK_EVAL(setup->surface != EGL_NO_SURFACE, error_surface, "Failed to create pbuffer surface!\n");

	// Create context
	setup->context = eglCreateContext(setup->display, config, EGL_NO_CONTEXT, contextAttributes);
	CHECK_EVAL(setup->context != EGL_NO_CONTEXT, error_context, "Failed to create context!\n");
	estatus = eglMakeCurrent(setup->display, setup->surface, setup->surface, setup->context);
	CHECK_EVAL(estatus, error_current, "Failed to make context current!\n");

	CHECK_EVAL((glerror = glGetError()) == GL_NO_ERROR, error_display, "GL error during init: error 0x%04x\n", glerror);

	return 0;

error_current:
	eglDestroyContext(setup->display, setup->context);
error_context:
	eglDestroySurface(setup->display, setup->surface);
error_surface:
	eglTerminate(setup->display);
error_display:
	return -1;
}

int setupEGLWindow(EGL_Setup *setup, EGL_Window *window, EGL_Setup *sharedSetup)
{
	EGLBoolean estatus;
	GLenum glerror;
	const EGLint configAttributes[] =
	{
		EGL_CONFORMANT,		EGL_OPENGL_ES2_BIT,
		EGL_SURFACE_TYPE,	EGL_WINDOW_BIT,
		EGL_RED_SIZE,		5,
		EGL_GREEN_SIZE,		6,
		EGL_BLUE_SIZE,		5,
		EGL_ALPHA_SIZE,		0,
		EGL_DEPTH_SIZE,		0,
		EGL_NONE
	};
	const EGLint contextAttributes[] =
	{
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};

	CHECK_EVAL((glerror = glGetError()) == GL_NO_ERROR, error_display, "GL error before init: error 0x%04x\n", glerror);

	// Get display
	setup->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	CHECK_EVAL(setup->display != EGL_NO_DISPLAY, error_display, "Failed to get EGL display!\n");
	estatus = eglInitialize(setup->display, &setup->versionMajor, &setup->versionMinor);
	CHECK_EVAL(estatus, error_surface, "Failed to initialized EGL!\n");

	// Choose config according to attributes
	EGLConfig config;
	EGLint numConfigs;
	estatus = eglChooseConfig(setup->display, configAttributes, &config, 1, &numConfigs);
	CHECK_EVAL(estatus, error_surface, "Failed to choose config!\n");

	// Bind OpenGL ES API (Needed?)
	estatus = eglBindAPI(EGL_OPENGL_ES_API);
	CHECK_EVAL(estatus, error_surface, "Failed to bind OpenGL ES API!\n");

	// Create surface
	setup->surface = eglCreateWindowSurface(setup->display, config, &window->eglWindow, NULL);
	CHECK_EVAL(setup->surface != EGL_NO_SURFACE, error_surface, "Failed to create window surface!\n");

	// Create context
	setup->context = eglCreateContext(setup->display, config, sharedSetup == NULL? EGL_NO_CONTEXT : sharedSetup->context, contextAttributes);
	CHECK_EVAL(setup->context != EGL_NO_CONTEXT, error_context, "Failed to create context!\n");
	estatus = eglMakeCurrent(setup->display, setup->surface, setup->surface, setup->context);
	CHECK_EVAL(estatus, error_current, "Failed to make context current!\n");

	CHECK_EVAL((glerror = glGetError()) == GL_NO_ERROR, error_display, "GL error during init: error 0x%04x\n", glerror);

	return 0;

error_current:
	eglDestroyContext(setup->display, setup->context);
error_context:
	eglDestroySurface(setup->display, setup->surface);
error_surface:
	eglTerminate(setup->display);
error_display:
	return -1;
}

void terminateEGL(EGL_Setup *setup)
{
	eglMakeCurrent(setup->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroyContext(setup->display, setup->context);
	eglDestroySurface(setup->display, setup->surface);
	eglTerminate(setup->display);
}

/* Create native window (basically just a rectangle on the screen we can render to */
int createWindow(EGL_Window *window)
{
	// Get display size
	uint32_t displayNum = 0; // Primary display
	uint32_t displayWidth, displayHeight;
	int32_t status = graphics_get_display_size(displayNum, &displayWidth, &displayHeight);
	if (status != 0) return status;

	// Setup Fullscreen
	VC_RECT_T destRect = {
		.width = (int32_t)displayWidth,
		.height = (int32_t)displayHeight,
	};
	VC_RECT_T srcRect = {
		.width = (int32_t)displayWidth << 16,
		.height = (int32_t)displayHeight << 16,
	};
	VC_DISPMANX_ALPHA_T alpha = {DISPMANX_FLAGS_ALPHA_FIXED_ALL_PIXELS, 255, 0};

	// Create window
	DISPMANX_DISPLAY_HANDLE_T display = vc_dispmanx_display_open(displayNum);
	DISPMANX_UPDATE_HANDLE_T update = vc_dispmanx_update_start(0);
	DISPMANX_ELEMENT_HANDLE_T element = vc_dispmanx_element_add(update, display, 0, &destRect, 0,
		 &srcRect, DISPMANX_PROTECTION_NONE, &alpha, NULL, DISPMANX_NO_ROTATE);
	int ret = vc_dispmanx_update_submit_sync(update);

	window->eglWindow.element = element;
	window->eglWindow.width = displayWidth;
	window->eglWindow.height = displayHeight;
	window->display = display;
	return ret;
}

int destroyWindow(EGL_Window *window)
{
	DISPMANX_UPDATE_HANDLE_T update = vc_dispmanx_update_start(0);
	vc_dispmanx_element_remove(update, window->eglWindow.element);
	int ret = vc_dispmanx_update_submit_sync(update);
	ret += vc_dispmanx_display_close(window->display);
	return ret;
}
