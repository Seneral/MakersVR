#include "eglUtil.h"

int setupEGL(EGL_Setup *setup, EGLNativeWindowType window)
{
	EGLBoolean estatus;
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

	GLenum glerror = glGetError();
	if (glerror != GL_NO_ERROR)
	{
		vcos_log_error("GL error before init: error 0x%04x", glerror);
		goto error_display;
	}

	// Get display
	setup->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	CHECK_EVAL(setup->display != EGL_NO_DISPLAY, "Failed to get EGL display!", error_display);
	estatus = eglInitialize(setup->display, &setup->versionMajor, &setup->versionMinor);
	CHECK_EVAL(estatus, "Failed to initialized EGL!", error_surface);

	// Choose config according to attributes
	EGLConfig config;
	EGLint numConfigs;
	estatus = eglChooseConfig(setup->display, configAttributes, &config, 1, &numConfigs);
	CHECK_EVAL(estatus, "Failed to choose config!", error_surface);

	// Bind OpenGL ES API (Needed?)
	estatus = eglBindAPI(EGL_OPENGL_ES_API);
	CHECK_EVAL(estatus, "Failed to bind OpenGL ES API!", error_surface);

	// Create surface
	setup->surface = eglCreateWindowSurface(setup->display, config, window, NULL);
	CHECK_EVAL(setup->surface != EGL_NO_SURFACE, "Failed to create window surface!", error_surface);

	// Create context
	setup->context = eglCreateContext(setup->display, config, EGL_NO_CONTEXT, contextAttributes);
	CHECK_EVAL(setup->context != EGL_NO_CONTEXT, "Failed to create context!", error_context);
	estatus = eglMakeCurrent(setup->display, setup->surface, setup->surface, setup->context);
	CHECK_EVAL(estatus, "Failed to make context current!", error_current);

	glerror = glGetError();
	if (glerror != GL_NO_ERROR)
	{
		vcos_log_error("GL error during init: error 0x%04x", glerror);
		goto error_current;
	}

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
int createNativeWindow(EGL_DISPMANX_WINDOW_T *window)
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

	window->element = element;
	window->width = displayWidth;
	window->height = displayHeight;
	vc_dispmanx_update_submit_sync(update);

	return 0;
}
