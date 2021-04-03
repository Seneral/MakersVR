#ifndef GCS_H
#define GCS_H

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GPU Camera Stream
	Simple MMAL camera stream using the preview port, keeping only the most recent camera frame buffer for realtime, low-latency CV applications
*/

/* Camera parameters passed for MMAL camera set up */
typedef struct GCS_CameraParams
{
	uint32_t mmalEnc;
	uint16_t width;
	uint16_t height;
	uint16_t fps;
	uint32_t shutterSpeed;
	uint32_t iso;
	uint8_t disableEXP;
	uint8_t digitalGain;
	uint8_t analogGain;
	uint8_t disableAWB;
	uint32_t disableISPBlocks;
} GCS_CameraParams;

/* Disable ISP Blocks */
// https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=175711
// (1 << 2); // Black Level Compensation
// (1 << 3); // Lens Shading
// (1 << 5); // White Balance Gain
// (1 << 7); // Defective Pixel Correction
// (1 << 9); // Crosstalk
// (1 << 18); // Gamma
// (1 << 22); // Sharpening
// (1 << 24); // Some Color Conversion

/* Opaque GPU Camera Stream structure */
typedef struct GCS GCS;

GCS *gcs_create(GCS_CameraParams *cameraParams);
void gcs_destroy(GCS *gcs);

/* Start GCS (camera stream). Enables MMAL camera and starts watchdog */
uint8_t gcs_start(GCS *gcs);

/* Stop GCS (camera output). Stops watchdog and disabled MMAL camera */
void gcs_stop(GCS *gcs);

/* Returns whether there is a new camera frame available */
uint8_t gcs_hasFrameBuffer(GCS *gcs);

/* Returns the most recent camera frame. If no camera frame is available yet, blocks until there is.
 * If the last frame has not been returned yet, returns NULL. */
void* gcs_requestFrameBuffer(GCS *gcs);

/* Returns the data of the given MMAL framebuffer. Use after gcs_requestFrameBuffer to get the underlying buffer. */
void* gcs_getFrameBufferData(void *framebuffer);

/* Return requested Frane Buffer after processing is done.
 * Has to be called before a new frame buffer can be requested. */
void gcs_returnFrameBuffer(GCS *gcs);


#ifdef __cplusplus
}
#endif

#endif
