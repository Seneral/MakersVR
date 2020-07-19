/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//#define BLOB_DEBUG
//#define BLOB_TRACE
//#define BLOB_VIZ_FOCUS
//#define BLOB_VIZ_DOTS
//#define MARKER_DEBUG

#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <termios.h>
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR

#include "applog.h"

#include "bcm_host.h"

#include "eglUtil.h"
#include "camGL.h"

#include "defines.hpp"
#include "mesh.hpp"
#include "shader.hpp"
#include "texture.hpp"

#include "blobdetection.hpp"

CamGL *camGL;
int dispWidth, dispHeight;
int camWidth = 1280, camHeight = 720, camFPS = 30;
float renderRatioCorrection;

EGL_Setup eglSetup;

struct termios terminalSettings;

static void setConsoleRawMode();
static void configSerialPort(int uartFD, uint32_t baud);


static uint8_t uartRcv[256];

// ID of connected UART device and acknowledgement of own ID
const uint8_t msg_id_own[] = u8"MakersVR_MarkerDetector";
const uint8_t msg_id_opp[] = u8"MakersVR_MarkerTracker";
const uint8_t msg_ack[] = u8"#ACK";
const uint8_t msg_nak[] = u8"#NAK";

inline void printBuffer(const uint8_t *buffer, uint8_t size)
{
	printf("0x");
	for (int i = 0; i < size; i++) printf("%02X", buffer[i]);
}

int main(int argc, char **argv)
{
	// ---- Read arguments ----

	CamGL_Params params = {
		.format = CAMGL_YUV,
		.width = (uint16_t)camWidth,
		.height = (uint16_t)camHeight,
		.fps = (uint16_t)camFPS,
		.shutterSpeed = 50,
		.iso = -1
	};

	int arg;
	while ((arg = getopt(argc, argv, "c:w:h:f:s:i:")) != -1)
	{
		switch (arg)
		{
			case 'c':
				if (strcmp(optarg, "YUV") == 0) params.format = CAMGL_YUV;
				else if (strcmp(optarg, "Y") == 0) params.format = CAMGL_Y;
				else if (strcmp(optarg, "RGB") == 0) params.format = CAMGL_RGB;
				break;
			case 's':
				params.shutterSpeed = std::stoi(optarg);
				break;
			case 'i':
				params.iso = std::stoi(optarg);
				break;
			case 'w':
				params.width = camWidth = std::stoi(optarg);
				break;
			case 'h':
				params.height = camHeight = std::stoi(optarg);
				break;
			case 'f':
				params.fps = camFPS = std::stoi(optarg);
				break;
			default:
				printf("Usage: %s [-c (RGB, Y, YUV)] [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso]\n", argv[0]);
				break;
		}
	}
	if (optind < argc - 1)
		printf("Usage: %s [-c (RGB, Y, YUV)] [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso]\n", argv[0]);
	if (params.shutterSpeed > 100)
	{
		printf("Blob detection requires low shutter speed (~8-100ns) to detect LEDs only. Too many light sources will blow up the CPU-side algorithm for connected component labeling and currently result in a crash.\n"); // Increase MAX_COMPONENTS in blobdetection.cpp if you really want to try
		params.shutterSpeed = 100;
	}

	// ---- Init ----

	// Init BCM Host
	bcm_host_init();

	// Create native window (not real GUI window)
	EGL_DISPMANX_WINDOW_T window;
	if (createNativeWindow(&window) != 0)
		return EXIT_FAILURE;
	dispWidth = window.width;
	dispHeight = window.height;
	renderRatioCorrection = (((float)dispHeight / camHeight) * camWidth) / dispWidth;

	// Setup EGL context
	setupEGL(&eglSetup, (EGLNativeWindowType*)&window);
	glClearColor(0.8f, 0.2f, 0.1f, 1.0f);

	// ---- Setup GL Resources ----

	initBlobDetection(camWidth, camHeight, eglSetup);
	CHECK_GL();

	// ---- Init UART Communication ----

	// Open UART port
	int uartFD = open("/dev/serial0", O_RDWR);
	if (uartFD < 0)
	{
		printf("Error %i from open: %s\n", errno, strerror(errno));
		return errno;
	}
	configSerialPort(uartFD, B115200); // B57600
	printf("Initiating UART connection...\n");

	// UART state
	bool rsp_id = false, rsp_ack = false;
	int timeout = 0;
	int rcvSize = 0;
	int lastCmd = 0;
	auto uart_clear = [&]()
	{
		timeout = 0;
		rcvSize = 0;
		lastCmd = 0;
	};
	auto uart_reset = [&]()
	{
		rsp_ack = false;
		rsp_id = false;
		uart_clear();
	};
	auto uart_wrID = [&]()
	{
		uint8_t idSz = sizeof(msg_id_own)-1;
		uint8_t header[] = { '#', 'I', (uint8_t)('0' + idSz/10), (uint8_t)('0' + idSz%10) };
		write(uartFD, header, sizeof(header));
		write(uartFD, msg_id_own, idSz);
		tcdrain(uartFD); // Wait for bytes to be sent
	};

	// Send own ID
	uart_wrID();

	while (true)
	{ // Identification loop to make sure UART communication works
		int num = read(uartFD, &uartRcv[rcvSize], sizeof(uartRcv)-rcvSize);
		rcvSize += num;
		if (num != 0)
			printf("Received %s!\n", uartRcv);

		while (uartRcv[lastCmd] != '#')
		{
			if (lastCmd >= rcvSize) break;
			lastCmd++;
		}

		if (uartRcv[lastCmd] == '#' && lastCmd+4 <= rcvSize)
		{
			if (uartRcv[lastCmd+1] == 'N' && uartRcv[lastCmd+2] == 'A' && uartRcv[lastCmd+3] == 'K')
			{ // NAK
				printf("NAK Received, resetting comm!\n");
				uart_reset();
				continue;
			}

			uint8_t cmdID = uartRcv[lastCmd+1];
			int dataCount = (uartRcv[lastCmd+2]-'0')*10 + (uartRcv[lastCmd+3]-'0');

			if (!rsp_ack || !rsp_id)
			{
				if (!rsp_ack && uartRcv[lastCmd+1] == 'A' && uartRcv[lastCmd+2] == 'C' && uartRcv[lastCmd+3] == 'K')
				{
					printf("Acknowledged by Marker Tracker!\n");
					rsp_ack = true;
					lastCmd += 4;
				}
				else if (!rsp_id && cmdID == 'I' && dataCount == sizeof(msg_id_opp)-1 && rcvSize >= lastCmd+4+dataCount)
				{
					if (memcmp(&uartRcv[lastCmd+4], msg_id_opp, dataCount) == 0)
					{
						printf("Identified Marker Tracker!\n");
						rsp_id = true;
						lastCmd += 4 + dataCount;
						write(uartFD, msg_ack, sizeof(msg_ack)-1);
						tcdrain(uartFD); // Wait for bytes to be sent
						if (!rsp_ack) uart_wrID();
					}
					else
					{
						printf("Failed to identify Marker Tracker!\n");
						uart_reset();
						write(uartFD, msg_nak, sizeof(msg_nak)-1);
						continue;
					}
				}
				if (rsp_id && rsp_ack)
				{ // Comm is ready
					printf("Comm is ready!\n");
					uart_clear();
					break;
				}
			}
		}

		if (rsp_ack || rsp_id) timeout++;
		if (timeout > 500)
		{ // Setup timeout, send NAK
			printf("Setup timeout exceeded, resetting comm!\n");
			uart_reset();
			write(uartFD, msg_nak, sizeof(msg_nak)-1);
			tcdrain(uartFD); // Wait for bytes to be sent
			continue;
		}

		sleep(0.001f);
	}

	// ---- Setup Camera ----

	// Init camera GL
	printf("Initializing Camera GL!\n");
	camGL = camGL_create(eglSetup, (const CamGL_Params*)&params);
	if (camGL == NULL)
	{
		printf("Failed to start Camera GL\n");
		terminateEGL(&eglSetup);
		return EXIT_FAILURE;
	}
	else
	{ // Finished setting up everything GL

		sleep(1);

		printf("Starting Camera GL!\n");
		int status = camGL_startCamera(camGL);
		if (status != CAMGL_SUCCESS)
		{
			printf("Failed to start camera GL with code %d!\n", status);
		}
		else
		{ // Process incoming frames

			// For non-blocking input even over ssh
			setConsoleRawMode();

			auto startTime = std::chrono::high_resolution_clock::now();
			auto lastTime = startTime;
			int numFrames = 0, lastFrames = 0;

			std::vector<Cluster> blobs;
			std::vector<uint8_t> blobReportBuffer;

			// Get handle to frame struct, stays the same when frames are updated
			CamGL_Frame *frame = camGL_getFrame(camGL);
			while ((status = camGL_nextFrame(camGL)) == CAMGL_SUCCESS)
			{ // Frames was available and has been processed

				// ---- Perform blob detection ----

				// Perform blob detection on frame and output results into both lists
				blobs.clear();
				performBlobDetection(frame, blobs);

				// ---- Visualize blob detection ----

				// Visualization view bounds
			#ifdef BLOB_VIZ_FOCUS
				Bounds viewBounds { camWidth, camHeight, 0, 0 };
				for (int i = 0; i < blobs.size(); i++)
				{
					Bounds b = blobs[i].bounds;
					viewBounds.minX = std::min(viewBounds.minX, b.minX - 50);
					viewBounds.maxX = std::max(viewBounds.maxX, b.maxX + 50);
					viewBounds.minY = std::min(viewBounds.minY, b.minY - 50);
					viewBounds.maxY = std::max(viewBounds.maxY, b.maxY + 50);
				}
				float viewRatio = (float)(viewBounds.maxX-viewBounds.minX)/(viewBounds.maxY-viewBounds.minY);
				float relRatio = viewRatio / ((float)dispWidth/dispHeight);
				if (relRatio > 1)
				{
					viewBounds.minX -= (relRatio-1) * (viewBounds.maxX-viewBounds.minX) / 2;
					viewBounds.maxX += (relRatio-1) * (viewBounds.maxX-viewBounds.minX) / 2;
				}
				if (relRatio < 1)
				{
					relRatio = 1/relRatio;
					viewBounds.minY -= (relRatio-1) * (viewBounds.maxY-viewBounds.minY) / 2;
					viewBounds.maxY += (relRatio-1) * (viewBounds.maxY-viewBounds.minY) / 2;
				}
			#else
				Bounds viewBounds { 0, 0, camWidth, camHeight };
			#endif

				// Visualisation
				glViewport((int)((1-renderRatioCorrection) * dispWidth / 2), 0, (int)(renderRatioCorrection * dispWidth), dispHeight);
				glBindFramebuffer(GL_FRAMEBUFFER, 0);
				// Visualize found points
				visualizeBlobDetection(blobs, viewBounds, (float)(viewBounds.maxX-viewBounds.minX)/dispWidth);
				// Submit frame
				eglSwapBuffers(eglSetup.display, eglSetup.surface);

				// ---- Send blob report over UART ----

				// Create blob report
				int blobReportLength = blobs.size()*6 + 4;
				blobReportBuffer.resize(blobReportLength);
				blobReportBuffer[0] = '#';
				blobReportBuffer[1] = 'B';
				blobReportBuffer[2] = '0' + blobs.size()/10;
				blobReportBuffer[3] = '0' + blobs.size()%10;

				// Fill with blob data
				uint16_t *blobData = (uint16_t*)&blobReportBuffer[4];
				for (int i = 0; i < blobs.size(); i++)
				{
					Cluster *blob = &blobs[i];
					uint8_t blobColor = 0b1111;
					int blobX = (int)((double)blob->centroid.X * 65536 / camWidth);
					int blobY = (int)((double)blob->centroid.Y * 65536 / camHeight);
					int blobS = std::min((int)(blob->centroid.S * 2), 255);
					blobData[i*3+0] = blobX;
					blobData[i*3+1] = blobY;
					blobData[i*3+2] = (blobS & 0xFF) | ((blobColor & 0xFF) << 8);
					/*int blobX = (int)((double)blob->centroid.X * 16383 / camWidth);
					int blobY = (int)((double)blob->centroid.Y * 16383 / camHeight);
					int blobS = std::min((int)(blob->centroid.S * 2), 255);
					uint32_t data = (blobX << 0) | (blobY << 14)
							|  << 22
							| (blobColor & ((1<<4)-1)) << 28;
					blobData[i] = data;
					*/
				}

				// Send blob report over UART
				write(uartFD, &blobReportBuffer[0], blobReportLength);

				// ---- Debugging and Statistics ----

				numFrames++;
				if (numFrames % 100 == 0)
				{ // Log FPS
					auto currentTime = std::chrono::high_resolution_clock::now();
					int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
					float elapsedS = (float)elapsedMS / 1000;
					lastTime = currentTime;
					int frames = (numFrames - lastFrames);
					lastFrames = numFrames;
					float fps = frames / elapsedS;
					int droppedFrames = 0;
					printf("%d frames over %.2fs (%.1ffps)! \n", frames, elapsedS, fps);
				}
				if (numFrames % 10 == 0)
				{ // Check for keys
					char cin;
					if (read(STDIN_FILENO, &cin, 1) == 1)
					{
						if (iscntrl(cin)) printf("%d", cin);
						else if (cin == 'q') break;
						else printf("%c", cin);
					}
				}
			}
			
			if (status != 0)
				printf("Camera GL was interrupted with code %d!\n", status);
			else
				camGL_stopCamera(camGL);
			close(uartFD);
		}
		camGL_destroy(camGL);
		terminateEGL(&eglSetup);

		return status == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
	}
}

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode()
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &terminalSettings);
		camGL_stopCamera(camGL);
	});
	termSet.c_lflag &= ~ECHO;
	termSet.c_lflag &= ~ICANON;
	termSet.c_cc[VMIN] = 0;
	termSet.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &termSet);
}

// Set the properties needed for the UART port
static void configSerialPort(int uartFD, uint32_t baud)
{
	struct termios tty;

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
	tty.c_cflag |= CS8; // 8 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON; // Disable canonical mode, where data is read line by line
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	// Set minimum bytes to wait for and timeout. Currently set to completely non-blocking
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be
	cfsetispeed(&tty, baud);
	cfsetospeed(&tty, baud);

	// Submit config
 	tcsetattr(uartFD, TCSANOW, &tty);
}
