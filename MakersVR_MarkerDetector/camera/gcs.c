//#include <stddef.h>
#include "gcs.h"

#include "interface/vcos/vcos_stdbool.h"
#include "interface/vcos/vcos_inttypes.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"

#define CHECK_STATUS_M(STATUS, MSG, ERRHANDLER) \
	if (STATUS != MMAL_SUCCESS) { \
		LOG_ERROR("%s: %s", MSG, mmal_status_to_string(STATUS)); \
		goto ERRHANDLER; \
	}
#define CHECK_STATUS_V(STATUS, MSG, ERRHANDLER) \
	if (STATUS != VCOS_SUCCESS) { \
		vcos_log_error(MSG); \
		goto ERRHANDLER; \
	}

/* Watchdog timeout - elapsed time to allow for no video frames received */
#define GCS_WATCHDOG_TIMEOUT_MS  4000

/* How many buffers the camera has to work with. 
 * 3 minimum, but might introduce some latency as only 2 can be used alternatingly in the background while one processes.
 * More than 4 are not needed and not used. */
#define GCS_SIMUL_BUFFERS 4

/* GPU Camera Stream
	Simple MMAL camera stream using the preview port, keeping only the most recent camera frame buffer for realtime, low-latency CV applications
	Handles MMAL component creation and setup

	Watchdog: Watches and stops stream if frames have stopped coming. Implemented by a timeout since last frame
	Buffer Pool: Collection of buffers used by the camera output to write to, processed and dropped frames are returned to it
*/
struct GCS
{
	// Flags
	uint8_t started; // Specifies that stream have started
	uint8_t error;

	// Camera parameters
	GCS_CameraParams cameraParams;

	MMAL_COMPONENT_T *camera; // Camera component
	MMAL_PORT_T *cameraOutput; // Camera output port (preview)
	MMAL_POOL_T *bufferPool; // Pool of buffers for camera output to use
	MMAL_BUFFER_HEADER_T *curFrameBuffer; // Nost recent camera frame buffer
	MMAL_BUFFER_HEADER_T *processingFrameBuffer; // Nost recent camera frame buffer
	VCOS_TIMER_T watchdogTimer; // Watchdog to detect if camera stops sending frames
	VCOS_MUTEX_T frameReadyMutex; // For waiting for next ready frame
};

/* Local functions (callbacks) */

// Camera control callback that receives events (incl. errors) of the MMAL camera
static void gcs_onCameraControl(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
// Camera output callback that receives new camera frames and updates the current frame
static void gcs_onCameraOutput(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
// Watchdog callback for when no new camera frames are pushed for a while, indicating an error
static void gcs_onWatchdogTrigger(void *context);


GCS *gcs_create(const GCS_CameraParams *cameraParams)
{
	// Temporary status return values
	MMAL_STATUS_T mstatus;
	VCOS_STATUS_T vstatus;

	LOG_TRACE("Creating GPU Camera Stream");

	// Allocate memory for structure
	GCS *gcs = vcos_calloc(1, sizeof(*gcs), "gcs");
	CHECK_STATUS_V((gcs ? VCOS_SUCCESS : VCOS_ENOMEM), "Failed to allocate context", error_allocate);
	gcs->cameraParams = *cameraParams;

	// Access mutex
	vstatus = vcos_mutex_create(&gcs->frameReadyMutex, "gcs-mutex");
	CHECK_STATUS_V(vstatus, "Failed to create mutex", error_mutex);

	// Setup timers and callbacks for watchdog (resets whenever a frame is received)
	vstatus = vcos_timer_create(&gcs->watchdogTimer, "gcs-watchdog-timer", gcs_onWatchdogTrigger, gcs);
	CHECK_STATUS_V(vstatus, "Failed to create timer", error_timer);

	// Create MMAL camera component
	mstatus = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &gcs->camera);
	CHECK_STATUS_M(mstatus, "Failed to create camera", error_cameraCreate);

	// Mess with the ISP blocks
	uint32_t disableISPBits = 0;
//	disableISPBits |= (1 << 2); // Black Level Compensation
//	disableISPBits |= (1 << 3); // Lens Shading
//	disableISPBits |= (1 << 5); // White Balance Gain
//	disableISPBits |= (1 << 7); // Defective Pixel Correction
//	disableISPBits |= (1 << 9); // Crosstalk
//	disableISPBits |= (1 << 18); // Gamma
//	disableISPBits |= (1 << 22); // Sharpening
	mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_CAMERA_ISP_BLOCK_OVERRIDE, ~disableISPBits);

	// Enable MMAL camera component
	mstatus = mmal_component_enable(gcs->camera);
	CHECK_STATUS_M(mstatus, "Failed to enable camera", error_cameraEnable);

	// Set camera parameters (See mmal_parameters_camera.h)
//	MMAL_PARAMETER_EXPOSUREMODE_T expMode;
//	expMode.hdr.id = MMAL_PARAMETER_EXPOSURE_MODE;
//	expMode.hdr.size = sizeof(MMAL_PARAMETER_EXPOSUREMODE_T);
//	expMode.value = MMAL_PARAM_EXPOSUREMODE_OFF;
//	mmal_port_parameter_set(gcs->camera->control, &expMode.hdr);
	if (gcs->cameraParams.shutterSpeed != 0) mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_SHUTTER_SPEED, gcs->cameraParams.shutterSpeed);
	if (gcs->cameraParams.iso >= 0) mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_ISO, (uint32_t)gcs->cameraParams.iso);
//	MMAL_PARAMETER_AWBMODE_T awbMode = { {MMAL_PARAMETER_AWB_MODE,sizeof(awbMode)}, MMAL_PARAM_AWBMODE_SUNLIGHT };
//	mmal_port_parameter_set(gcs->camera->control, &awbMode.hdr);
	// Enable MMAL camera port
	gcs->camera->control->userdata = (struct MMAL_PORT_USERDATA_T *)gcs;
	mstatus = mmal_port_enable(gcs->camera->control, gcs_onCameraControl);
	CHECK_STATUS_M(mstatus, "Failed to enable camera control port", error_portEnable);
	gcs->cameraOutput = gcs->camera->output[0]; // Preview Port 0

	// Set format of video output
	MMAL_ES_FORMAT_T *format = gcs->cameraOutput->format;
	format->encoding = gcs->cameraParams.mmalEnc == 0? MMAL_ENCODING_OPAQUE : gcs->cameraParams.mmalEnc;
	format->encoding_variant = MMAL_ENCODING_I420;
	MMAL_VIDEO_FORMAT_T *videoFormat = &format->es->video;
	videoFormat->width = gcs->cameraParams.width;
	videoFormat->height = gcs->cameraParams.height;
	videoFormat->crop.x = 0;
	videoFormat->crop.y = 0;
	videoFormat->crop.width = gcs->cameraParams.width;
	videoFormat->crop.height = gcs->cameraParams.height;
	videoFormat->frame_rate.num = gcs->cameraParams.fps;
	videoFormat->frame_rate.den = 1;
	mstatus = mmal_port_format_commit(gcs->cameraOutput);
	CHECK_STATUS_M(mstatus, "Failed to set output port format", error_portEnable);

	// Enable zero-copy to store buffers in shared memory
	mstatus = mmal_port_parameter_set_boolean(gcs->cameraOutput, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	CHECK_STATUS_M((mstatus == MMAL_ENOSYS ? MMAL_SUCCESS : mstatus), "Failed to enable zero copy", error_portEnable);

	// Set buffer num/size
	gcs->cameraOutput->buffer_num = GCS_SIMUL_BUFFERS;//gcs->cameraOutput->buffer_num_recommended;
	gcs->cameraOutput->buffer_size = gcs->cameraOutput->buffer_size_recommended;

	// Setup buffer pool for camera output port to use (after enabling zero-copy so those buffers will be allocated through VCSM)
	gcs->bufferPool = mmal_port_pool_create(gcs->cameraOutput, gcs->cameraOutput->buffer_num, gcs->cameraOutput->buffer_size);
	CHECK_STATUS_M((gcs->bufferPool ? MMAL_SUCCESS : MMAL_ENOMEM), "Error allocating pool", error_pool);

	LOG_TRACE("Finished setup of GCS");

	return gcs;

error_pool:
	mmal_port_disable(gcs->cameraOutput);
error_portEnable:
	mmal_component_disable(gcs->camera);
error_cameraEnable:
	mmal_component_destroy(gcs->camera);
error_cameraCreate:
	vcos_timer_delete(&gcs->watchdogTimer);
error_timer:
	vcos_mutex_delete(&gcs->frameReadyMutex);
error_mutex:
	vcos_free(gcs);
error_allocate:
	return NULL;
}

void gcs_destroy(GCS *gcs)
{
	if (!gcs) return;

	// Stop worker thread, disable camera component
	gcs_stop(gcs);

	// Destroy camera component
	mmal_component_disable(gcs->camera);
	mmal_component_destroy(gcs->camera);

	// Free remaining resources
	mmal_pool_destroy(gcs->bufferPool);
	vcos_mutex_delete(&gcs->frameReadyMutex);
	vcos_timer_delete(&gcs->watchdogTimer);
	vcos_free(gcs);
}

/* Start GCS (camera stream). Enables MMAL camera and starts watchdog */
uint8_t gcs_start(GCS *gcs)
{
	// Ensure GCS is stopped first
	gcs_stop(gcs);
	gcs->error = 0;
	gcs->started = 1;

	// Lock mutex to signal frame is not ready
	vcos_mutex_lock(&gcs->frameReadyMutex);

	// Enable camera output port and set callback to receive camera frame buffers
	gcs->cameraOutput->userdata = (struct MMAL_PORT_USERDATA_T *)gcs;
	MMAL_STATUS_T mstatus = mmal_port_enable(gcs->cameraOutput, gcs_onCameraOutput);
	CHECK_STATUS_M(mstatus, "Failed to enable output port", error_port);

	// Send unused buffers to video port to use
	MMAL_BUFFER_HEADER_T *buffer;
	while ((buffer = mmal_queue_get(gcs->bufferPool->queue)) != NULL)
	{
		MMAL_STATUS_T mstatus = mmal_port_send_buffer(gcs->cameraOutput, buffer);
		if (mstatus != MMAL_SUCCESS)
			LOG_ERROR("Failed to send buffer to %s", gcs->cameraOutput->name);
	}

	// Start watchdog timer that may stop stream due to lack of frames received (resets whenever frame is received)
	vcos_timer_set(&gcs->watchdogTimer, GCS_WATCHDOG_TIMEOUT_MS);

	return 0;

error_port:
	return -1;
}

/* Stop GCS (camera output). Stops watchdog and disabled MMAL camera */
void gcs_stop(GCS *gcs)
{
	gcs->started = 0;

	// Stop running timers
	vcos_timer_cancel(&gcs->watchdogTimer);

	if (gcs->started)
	{
		// Disable camera output
		mmal_port_disable(gcs->cameraOutput);

		// Stop potentially waiting user
		if (vcos_mutex_is_locked(&gcs->frameReadyMutex))
			vcos_mutex_unlock(&gcs->frameReadyMutex);

		// Reset unused frames
		if (gcs->processingFrameBuffer)
		{
			mmal_buffer_header_release(gcs->processingFrameBuffer);
			gcs->processingFrameBuffer = NULL;
		}
		if (gcs->curFrameBuffer)
		{
			mmal_buffer_header_release(gcs->curFrameBuffer);
			gcs->curFrameBuffer = NULL;
		}
	}
}

/* Returns whether there is a new camera frame available */
uint8_t gcs_hasFrameBuffer(GCS *gcs)
{
	return gcs->curFrameBuffer != NULL;
}

/* Returns the most recent camera frame. If no camera frame is available yet, blocks until there is.
 * If the last frame has not been returned yet, returns NULL. */
void* gcs_requestFrameBuffer(GCS *gcs)
{
	vcos_mutex_lock(&gcs->frameReadyMutex);
	if (gcs->processingFrameBuffer)
	{ // Not cleaned up last frame
		LOG_ERROR("Not cleaned up last fraame!");
		return NULL;
	}
	gcs->processingFrameBuffer = gcs->curFrameBuffer;
	gcs->curFrameBuffer = NULL;
	return gcs->processingFrameBuffer;
}

/* Returns the data of the given MMAL framebuffer. Use after gcs_requestFrameBuffer to get the underlying buffer. */
void* gcs_getFrameBufferData(void *framebuffer)
{
	return ((MMAL_BUFFER_HEADER_T*)framebuffer)->data;
}

/* Return requested Frane Buffer after processing is done.
 * Has to be called before a new frame buffer can be requested. */
void gcs_returnFrameBuffer(GCS *gcs)
{
	if (gcs->processingFrameBuffer)
	{
		mmal_buffer_header_release(gcs->processingFrameBuffer);
		gcs->processingFrameBuffer = NULL;
	}
}

/** Callback from the camera control port. */
static void gcs_onCameraControl(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buf)
{
	GCS *gcs = (GCS *)port->userdata;
	if (buf->cmd == MMAL_EVENT_ERROR)
	{
		LOG_ERROR("%s: MMAL error: %s", port->name, mmal_status_to_string(*(MMAL_STATUS_T *)buf->data));
		gcs_stop(gcs);
	}
	else
	{
		LOG_TRACE("%s: buf %p, event %4.4s", port->name, buf, (char *)&buf->cmd);
	}
	mmal_buffer_header_release(buf);
}

/** Callback from camera output port - receive buffer and replace as current */
static void gcs_onCameraOutput(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	GCS *gcs = (GCS *)port->userdata;
	if (buffer->length == 0)
	{
		LOG_TRACE("%s: zero-length buffer => EOS", port->name);
		mmal_buffer_header_release(buffer);
	}
	else if (buffer->data == NULL)
	{
		LOG_ERROR("%s: zero buffer handle", port->name);
		mmal_buffer_header_release(buffer);
	}
	else if (!gcs->started)
	{
		mmal_buffer_header_release(buffer);
	}
	else
	{
		// Reset watchdog timer for detecting when frames stop coming
		vcos_timer_set(&gcs->watchdogTimer, GCS_WATCHDOG_TIMEOUT_MS);

		// Retract frame ready signal during switch
		vcos_mutex_trylock(&gcs->frameReadyMutex);

		// Release now outdated camera frame
		if (gcs->curFrameBuffer != NULL)
		{ // If it does not exist, it has been consumed
			mmal_buffer_header_release(gcs->curFrameBuffer);
			gcs->curFrameBuffer = NULL;
			// On drop frame
		}

		// Set the newest camera frame
		gcs->curFrameBuffer = buffer;

		// Send buffer back to port for use (needed? it's a port buffer, should automatically do it, right?)
		while ((buffer = mmal_queue_get(gcs->bufferPool->queue)) != NULL)
		{
			MMAL_STATUS_T status = mmal_port_send_buffer(gcs->cameraOutput, buffer);
			if (status != MMAL_SUCCESS)
				LOG_ERROR("Failed to send buffer to %s", gcs->cameraOutput->name);
		}
	}

	// If not done already signal that a frame is ready
	if (vcos_mutex_is_locked(&gcs->frameReadyMutex))
		vcos_mutex_unlock(&gcs->frameReadyMutex);
}

/** Watchdog timer callback - stops playback because no frames have arrived from the camera for a while */
static void gcs_onWatchdogTrigger(void *context)
{
	GCS *gcs = context;
	LOG_ERROR("%s: no frames received for %d ms, aborting", gcs->cameraOutput->name, GCS_WATCHDOG_TIMEOUT_MS);
	gcs_stop(gcs);
}
