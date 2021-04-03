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
#define GCS_WATCHDOG_TIMEOUT_MS 2000

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


GCS *gcs_create(GCS_CameraParams *cameraParams)
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

	// Get camera Info
	MMAL_COMPONENT_T *cameraInfoComp;
	mstatus = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &cameraInfoComp);
	CHECK_STATUS_M(mstatus, "Failed to create camera info component", error_cameraCreate);
	MMAL_PARAMETER_CAMERA_INFO_T camInfo = { { MMAL_PARAMETER_CAMERA_INFO, sizeof(camInfo) }, 0 };
	mstatus = mmal_port_parameter_get(cameraInfoComp->control, &camInfo.hdr);
	CHECK_STATUS_M(mstatus, "Failed to get camera info", error_cameraCreate);
	// Red out camera info
	const int cameraNum = 0;
	int maxWidth = 0, maxHeight = 0;
	if (camInfo.num_cameras > 0)
	{
		maxWidth = camInfo.cameras[cameraNum].max_width;
		maxHeight = camInfo.cameras[cameraNum].max_height;
		LOG_ERROR("Camera '%s' has sensor res %dx%d", camInfo.cameras[cameraNum].camera_name, maxWidth, maxHeight);
	}
	mstatus = mmal_component_destroy(cameraInfoComp);
	CHECK_STATUS_M(mstatus, "Failed to destroy camera info component", error_cameraCreate);

	// Create MMAL camera component
	mstatus = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &gcs->camera);
	CHECK_STATUS_M(mstatus, "Failed to create camera", error_cameraCreate);

	// Make sure camera has outputs
	if (!gcs->camera->output_num)
	{
		LOG_ERROR("Camera doesn't have any output ports!");
		goto error_portEnable;
	}

	// Set camera mode
//	mstatus = mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, 0);
//	CHECK_STATUS_M(mstatus, "Failed to set camera mode", error_portEnable);

	// Enable MMAL camera port
	gcs->camera->control->userdata = (struct MMAL_PORT_USERDATA_T *)gcs;
	mstatus = mmal_port_enable(gcs->camera->control, gcs_onCameraControl);
	CHECK_STATUS_M(mstatus, "Failed to enable camera control port", error_portEnable);

	MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
	{
		{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
		.max_stills_w = gcs->cameraParams.width,
		.max_stills_h = gcs->cameraParams.height,
		.stills_yuv422 = 0,
		.one_shot_stills = 0,
		.max_preview_video_w = gcs->cameraParams.width,
		.max_preview_video_h = gcs->cameraParams.height,
		.num_preview_video_frames = 3,
		.stills_capture_circular_buffer_height = 0,
		.fast_preview_resume = 0,
		.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
	};
	mstatus = mmal_port_parameter_set(gcs->camera->control, &cam_config.hdr);
	CHECK_STATUS_M(mstatus, "Failed to set camera config", error_portEnable);


	gcs->cameraOutput = gcs->camera->output[1]; // Select port to use

	// Set format of video output
	MMAL_ES_FORMAT_T *format = gcs->cameraOutput->format;
	format->encoding = gcs->cameraParams.mmalEnc == 0? MMAL_ENCODING_OPAQUE : gcs->cameraParams.mmalEnc;
	format->encoding_variant = MMAL_ENCODING_I420;
	MMAL_VIDEO_FORMAT_T *videoFormat = &format->es->video;
	videoFormat->width = VCOS_ALIGN_UP(gcs->cameraParams.width, 32);
	videoFormat->height = VCOS_ALIGN_UP(gcs->cameraParams.height, 16);
	videoFormat->crop.x = 0;
	videoFormat->crop.y = 0;
	videoFormat->crop.width = gcs->cameraParams.width;
	videoFormat->crop.height = gcs->cameraParams.height;
	videoFormat->frame_rate.num = gcs->cameraParams.fps;
	videoFormat->frame_rate.den = 1;
	mstatus = mmal_port_format_commit(gcs->cameraOutput);
	CHECK_STATUS_M(mstatus, "Failed to set output port format", error_cameraEnable);

	// Enable zero-copy to store buffers in shared memory
	mstatus = mmal_port_parameter_set_boolean(gcs->cameraOutput, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	CHECK_STATUS_M((mstatus == MMAL_ENOSYS ? MMAL_SUCCESS : mstatus), "Failed to enable zero copy", error_cameraEnable);

	// Set buffer num/size
	gcs->cameraOutput->buffer_num = GCS_SIMUL_BUFFERS;//gcs->cameraOutput->buffer_num_recommended;
	gcs->cameraOutput->buffer_size = gcs->cameraOutput->buffer_size_recommended;

	// Set used ISP blocks
	// https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=175711
	mstatus += mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_CAMERA_ISP_BLOCK_OVERRIDE, ~cameraParams->disableISPBlocks);
	CHECK_STATUS_M(mstatus, "Failed to disable selected ISP blocks", error_cameraEnable);

	// Enable MMAL camera component
	mstatus = mmal_component_enable(gcs->camera);
	CHECK_STATUS_M(mstatus, "Failed to enable camera", error_cameraEnable);

	// Set camera parameters (See mmal_parameters_camera.h)
	mstatus += mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_ISO, (uint32_t)gcs->cameraParams.iso);
	CHECK_STATUS_M(mstatus, "Failed to set camera parameters (ISO)", error_pool);

	if (gcs->cameraParams.disableEXP)
	{ // Fix Exposure to set ISO value
		MMAL_PARAMETER_EXPOSUREMODE_T expMode = { { MMAL_PARAMETER_EXPOSURE_MODE, sizeof(expMode) }, MMAL_PARAM_EXPOSUREMODE_OFF };
		mstatus += mmal_port_parameter_set(gcs->camera->control, &expMode.hdr);
		mstatus += mmal_port_parameter_set_rational(gcs->camera->control, MMAL_PARAMETER_ANALOG_GAIN, (MMAL_RATIONAL_T){ gcs->cameraParams.analogGain*65536, 65536 });
		mstatus += mmal_port_parameter_set_rational(gcs->camera->control, MMAL_PARAMETER_DIGITAL_GAIN, (MMAL_RATIONAL_T){ gcs->cameraParams.digitalGain*65536, 65536 });
	}
	CHECK_STATUS_M(mstatus, "Failed to set camera parameters (EXP)", error_pool);

	if (gcs->cameraParams.disableAWB)
	{ // Fix AWB to constant 1,1 gain
		MMAL_PARAMETER_AWBMODE_T awbMode = { { MMAL_PARAMETER_AWB_MODE, sizeof(awbMode) }, MMAL_PARAM_AWBMODE_OFF };
		mstatus += mmal_port_parameter_set(gcs->camera->control, &awbMode.hdr);
		float rGain = 1.0f, bGain = 1.0f;
		MMAL_PARAMETER_AWB_GAINS_T awbGains = { { MMAL_PARAMETER_CUSTOM_AWB_GAINS, sizeof(awbGains) }, { (uint32_t)(rGain*65536), 65536 }, { (uint32_t)(bGain*65536), 65536 }};
		mstatus += mmal_port_parameter_set(gcs->camera->control, &awbGains.hdr);
	}
	CHECK_STATUS_M(mstatus, "Failed to set camera parameters (AWB)", error_pool);

	mstatus += mmal_port_parameter_set_uint32(gcs->camera->control, MMAL_PARAMETER_SHUTTER_SPEED, gcs->cameraParams.shutterSpeed);
	CHECK_STATUS_M(mstatus, "Failed to set camera parameters (SS)", error_pool);

	// Setup buffer pool for camera output port to use (after enabling zero-copy so those buffers will be allocated through VCSM)
	gcs->bufferPool = mmal_port_pool_create(gcs->cameraOutput, gcs->cameraOutput->buffer_num, gcs->cameraOutput->buffer_size);
	CHECK_STATUS_M((gcs->bufferPool ? MMAL_SUCCESS : MMAL_ENOMEM), "Error allocating pool", error_pool);

	LOG_TRACE("Finished setup of GCS");

	return gcs;

error_pool:
	mmal_component_disable(gcs->camera);
error_cameraEnable:
	mmal_port_disable(gcs->camera->control);
error_portEnable:
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
	mmal_pool_destroy(gcs->bufferPool); // TODO: Needed? raspividyuv doesn't do this
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

	mstatus = mmal_port_parameter_set_boolean(gcs->cameraOutput, MMAL_PARAMETER_CAPTURE, true);
	CHECK_STATUS_M(mstatus, "Failed to set capture", error_port);

	// Start watchdog timer that may stop stream due to lack of frames received (resets whenever frame is received)
	vcos_timer_set(&gcs->watchdogTimer, GCS_WATCHDOG_TIMEOUT_MS);

	return 0;

error_port:
	return -1;
}

/* Stop GCS (camera output). Stops watchdog and disabled MMAL camera */
void gcs_stop(GCS *gcs)
{
	// Stop running timers
	vcos_timer_cancel(&gcs->watchdogTimer);

	if (gcs->started)
	{
		mmal_port_parameter_set_boolean(gcs->cameraOutput, MMAL_PARAMETER_CAPTURE, false);

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

	gcs->started = 0;
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
	if (!gcs->processingFrameBuffer)
	{ // Not cleaned up last frame
		LOG_ERROR("No current frame buffer!");
		return NULL;
	}
	mmal_buffer_header_mem_lock(gcs->processingFrameBuffer); // TODO: Needed?
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
		mmal_buffer_header_mem_unlock(gcs->processingFrameBuffer); // TODO: Needed?
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
		LOG_ERROR("%s: buf %p, event %4.4s", port->name, buf, (char *)&buf->cmd);
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
