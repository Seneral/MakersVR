/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

//#define UART_COMM
//#define BLOB_VIZ_FOCUS
//#define MARKER_DEBUG

#include <vector>
#include <chrono>
// Console and UART
#include <termios.h>
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR
// Threading
#include <thread>
#include <mutex>
#include <atomic>

// System interface
#include "bcm_host.h"

// Camera and general GL
#include "eglUtil.h"
#include "camGL.h"
#include "mesh.hpp"
#include "shader.hpp"
#include "texture.hpp"

// GL Blob detection
#include "blobdetection.hpp"


/* Variables */

// Main camera and GL setup
CamGL *camGL;
CamGL_Params params;
EGL_Setup eglSetup;
EGL_Setup eglDisplaySetup;

bool displayViz;
bool useUART;

// Terminal Output and Input
struct termios terminalSettings;
static void setConsoleRawMode();

// UART
static void configSerialPort(int uartFD, uint32_t baud);

// UART state
static bool rsp_id, rsp_ack;
static int timeout;
static int rcvSize;
static int lastCmd;
static uint8_t cmdNAK;
static uint8_t cmdACK;
static uint8_t cmdID;
static uint8_t cmdSz;
static int cmdPos;
static bool cmdValid;
static uint8_t uartRcv[256];

// ID of connected UART device and acknowledgement of own ID
const uint8_t msg_id_own_full[] = u8"#I17MakersVR_MarkerDetector"; // Assert length of payload in hex
const uint8_t msg_id_opp[] = u8"MakersVR_MarkerTracker";
const uint8_t msg_ack[] = u8"#ACK";
const uint8_t msg_nak[] = u8"#NAK";
const uint8_t msg_ping[] = u8"#P00";
const uint8_t hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
#define UART_PING_TIMEOUT 1500

// Thread Management
static void GPUBlobDetectionThread();
static void CPUBlobDetectionThread();
static void VisualizationThread();
std::thread GPUThread;
std::thread CPUThread;
std::thread VizThread;
std::atomic<bool> blobDetectionRunning;
std::mutex blobmaskReady; // CPU part done accessing blob mask, GPU part ready to go
std::mutex blobmaskDone; // GPU blob mask done
std::mutex blobsReady; // Uart and Viz finished, blobs can be accessed
std::mutex blobsDone; // Blobs have been updated
std::mutex vizReady; // Viz can start after blob update
std::mutex vizDone; // Viz is finished accessing blobs

// Current and next frame's blobs
std::vector<Cluster> blobs;
std::vector<Cluster> blobsWIP;


/* Functions */

inline void printBuffer(const uint8_t *buffer, uint8_t size)
{
	printf("0x");
	for (int i = 0; i < size; i++) printf("%02X", buffer[i]);
}

inline void uart_clear()
{
	rcvSize = 0;
	lastCmd = 0;
}

inline void uart_reset()
{
	cmdValid = true;
	timeout = 0;
	rsp_ack = false;
	rsp_id = false;
	uart_clear();
}

inline void uart_clean()
{ // Called after command has been handled
	if (lastCmd > rcvSize-1 || (lastCmd == rcvSize-1 && uartRcv[lastCmd] != '#'))
	{ // At end of buffer and not waiting for a full command
		uart_clear(); // Discard buffer and start from beginning
		// This will still allow the current command to be read until next frame
	}
}

inline bool uart_rcvCmd(int fd)
{
	// Receive new data
	int num = read(fd, &uartRcv[rcvSize], sizeof(uartRcv)-rcvSize);
	if (lastCmd >= rcvSize-1 && num <= 0) return false;
//	printf("UART Rcv: '%.*s'\n", rcvSize-lastCmd, uartRcv+lastCmd);
	rcvSize += num;
	// Skip stray # incase cmdID has not been validated
	if (!cmdValid) lastCmd++;
	cmdValid = true;
	// Locate next command
	while (lastCmd < rcvSize-1 && uartRcv[lastCmd] != '#')
		lastCmd++;
	// If no command is found, clear to reset buffer
	if (uartRcv[lastCmd] != '#')
	{
		uart_clear();
		return false;
	}
	// Make sure the full command header is received
	if (lastCmd+4 > rcvSize) return false;
	// Identify NAK and ACK
	cmdNAK = uartRcv[lastCmd+1] == 'N' && uartRcv[lastCmd+2] == 'A' && uartRcv[lastCmd+3] == 'K';
	cmdACK = uartRcv[lastCmd+1] == 'A' && uartRcv[lastCmd+2] == 'C' && uartRcv[lastCmd+3] == 'K';
	if (cmdNAK || cmdACK)
	{ // Accept basic command
		lastCmd += 4; // Advance to next command
		uart_clean(); // Check if buffer can be reset
		cmdValid = true; // Validate command, don't skip
	}
	else
	{ // General command requires validation
		cmdID = uartRcv[lastCmd+1];
		uint_fast8_t a = uartRcv[lastCmd+2]-'0', b = uartRcv[lastCmd+3]-'0';
		a = a > 9? a-('A'-'9'+1) : a;
		b = b > 9? b-('A'-'9'+1) : b;
		cmdSz = a*16 + b;//(uartRcv[lastCmd+2]-'0')*10 + (uartRcv[lastCmd+3]-'0');
		cmdValid = false; // If cmdID not validated, will skip
	}
	return true;
}

inline bool uart_fetchCmd()
{ // Command ID is valid, wait until it's received in full
	cmdValid = true;
	if (rcvSize >= lastCmd+4+cmdSz)
	{ // Received in full
		cmdPos = lastCmd+4; // Save data pos
		lastCmd += 4+cmdSz; // Advance to after command
		uart_clean(); // Check if buffer can be reset
		return true;
	}
	return false;
}

int main(int argc, char **argv)
{
	// ---- Read arguments ----

	params = {
		.format = CAMGL_Y,
		.width = 640,
		.height = 480,
		.fps = 30,
		.shutterSpeed = 50,
		.iso = -1
	};
	displayViz = false;
	useUART = false;

	int arg;
	while ((arg = getopt(argc, argv, "c:w:h:f:s:i:du")) != -1)
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
				params.width = std::stoi(optarg);
				break;
			case 'h':
				params.height = std::stoi(optarg);
				break;
			case 'f':
				params.fps = std::stoi(optarg);
				break;
			case 'd':
				displayViz = true;
				break;
			case 'u':
				useUART = true;
				break;
			default:
				printf("Usage: %s [-c (RGB, Y, YUV)] [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso] [-d display visualization] [-u Enable UART]\n", argv[0]);
				break;
		}
	}
	if (optind < argc - 1)
		printf("Usage: %s [-c (RGB, Y, YUV)] [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso] [-d display visualization] [-u Enable UART]\n", argv[0]);
	if (params.shutterSpeed > 1000)
		printf("Shutter speed unusually high for blob detection.\n");

	// ---- Init ----

	// Init BCM Host
	bcm_host_init();

	// For non-blocking input even over ssh
	setConsoleRawMode();

	bool running = true;
	int uartFD;
	bool identified;
	auto startTime = std::chrono::high_resolution_clock::now();
	auto lastTime = startTime;
	auto lastPing = startTime;
	int numFrames, lastFrames;
	std::vector<uint8_t> blobReportBuffer;

	while (running)
	{

		// ---- Init UART Communication ----

		// Open UART port
		if (useUART)
		{
			uartFD = open("/dev/serial0", O_RDWR);
			if (uartFD < 0)
			{
				printf("Error: Can't open UART port! (%i: %s)\n", errno, strerror(errno));
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				continue;
			}
			configSerialPort(uartFD, B115200); // B57600
			uart_reset();
			printf("Initiating UART connection...\n");
		}
		else
		{
			goto phase_blobdetection;
		}

		/* Identification Step */

phase_identification:

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		timeout = 0;

		// Send own ID
		write(uartFD, msg_id_own_full, sizeof(msg_id_own_full)-1);
		tcdrain(uartFD); // Wait for bytes to be sent

		while (true)
		{ // Identification loop to make sure UART communication works

			if (uart_rcvCmd(uartFD))
			{ // Got a new command to handle
				if (cmdNAK)
				{ // NAK received
					printf("NAK Received, resetting comm!\n");
					uart_reset();
					goto phase_identification;
				}
				else if (cmdACK && !rsp_ack)
				{ // ACK received
					printf("Acknowledged by Marker Tracker!\n");
					rsp_ack = true;
				}
				else if (cmdID == 'I' && uart_fetchCmd() && !rsp_id)
				{ // Received full id command
					if (cmdSz == sizeof(msg_id_opp)-1 && memcmp(&uartRcv[cmdPos], msg_id_opp, cmdSz) == 0)
					{ // Proper identity
						printf("Identified Marker Tracker!\n");
						rsp_id = true;
						write(uartFD, msg_ack, sizeof(msg_ack)-1);
						tcdrain(uartFD); // Wait for bytes to be sent
						if (!rsp_ack) // Send own ID
							write(uartFD, msg_id_own_full, sizeof(msg_id_own_full)-1);
					}
					else
					{ // Wrong identity
						printf("Failed to identify Marker Tracker!\n");
						write(uartFD, msg_nak, sizeof(msg_nak)-1);
						tcdrain(uartFD); // Wait for bytes to be sent
						uart_reset();
						goto phase_identification;
					}
				}
				if (rsp_id && rsp_ack)
				{ // Comm is ready
					printf("Comm is ready!\n");
					uart_clear();
					goto phase_setup;
				}

			}

			// Count timeout from first interaction
			if (rsp_ack || rsp_id) timeout++;
			if (timeout > 500)
			{ // Setup timeout, send NAK
				printf("Identification timeout exceeded, resetting comm!\n");
				write(uartFD, msg_nak, sizeof(msg_nak)-1);
				tcdrain(uartFD); // Wait for bytes to be sent
				uart_reset();
				goto phase_identification;
			}

			char cin;
			if (read(STDIN_FILENO, &cin, 1) == 1)
			{
				if (iscntrl(cin)) printf("%d", cin);
				else if (cin == 'q')
				{ // Complete stop of program requested
					running = false;
					goto abort;
				}
				else printf("%c", cin);
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}


		/* Setup phase */

phase_setup:

		lastPing = std::chrono::high_resolution_clock::now();
		while (true)
		{ // Setup packets and start blob detection command

			if (uart_rcvCmd(uartFD))
			{ // Got a new command to handle
				if (cmdNAK)
				{ // NAK received
					printf("NAK Received, resetting comm!\n");
					uart_reset();
					goto phase_identification;
				}
				else if (cmdID == '+' && uart_fetchCmd())
				{ // Received full start command
					printf("Received START command!\n");
					goto phase_blobdetection;
				}
				else if (cmdID == 'S' && uart_fetchCmd())
				{ // Received setup packet
					params.width = *(uint16_t*)&uartRcv[cmdPos+0];
					params.height = *(uint16_t*)&uartRcv[cmdPos+2];
					params.fps = *(uint16_t*)&uartRcv[cmdPos+4];
					params.shutterSpeed = *(uint16_t*)&uartRcv[cmdPos+6];
					printf("Received setup: %dx%d @ %d fps, %dns shutter speed\n", params.width, params.height, params.fps, params.shutterSpeed);
				}
				else if (cmdID == 'P' && uart_fetchCmd())
				{ // Received ping, answer ping
					write(uartFD, msg_ping, sizeof(msg_ping)-1);
					lastPing = std::chrono::high_resolution_clock::now();
				}
			}

			// Check timeout
			auto currentTime = std::chrono::high_resolution_clock::now();
			int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPing).count();
			if (elapsedMS > UART_PING_TIMEOUT)
			{ // Setup timeout, send NAK
				printf("Setup timeout exceeded, resetting comm!\n");
				write(uartFD, msg_nak, sizeof(msg_nak)-1);
				tcdrain(uartFD); // Wait for bytes to be sent
				uart_reset();
				goto phase_identification;
			}

			// Check input
			char cin;
			if (read(STDIN_FILENO, &cin, 1) == 1)
			{
				if (iscntrl(cin)) printf("%d", cin);
				else if (cin == 'q')
				{ // Complete stop of program requested
					running = false;
					goto abort;
				}
				else printf("%c", cin);
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		/* Blob Detection phase */

phase_blobdetection:

		printf("Starting blob detection!\n");

		// Initialize thread state
		blobDetectionRunning = true;
		blobmaskReady.unlock();
		blobmaskDone.try_lock();
		blobsReady.unlock();
		blobsDone.try_lock();
		vizReady.try_lock();
		vizDone.try_lock();

		// Start the threads
		GPUThread = std::thread(GPUBlobDetectionThread);
		CPUThread = std::thread(CPUBlobDetectionThread);
		if (displayViz) VizThread = std::thread(VisualizationThread);

		// Debug
		startTime = std::chrono::high_resolution_clock::now();
		lastTime = startTime;
		numFrames = 0;
		lastFrames = 0;

		while (blobDetectionRunning)
		{
			if (useUART && uart_rcvCmd(uartFD))
			{ // Got a new command to handle
				if (cmdNAK)
				{ // NAK received
					printf("NAK Received, resetting comm!\n");
					uart_reset();
					break;
				}
				else if (cmdID == '-' && uart_fetchCmd())
				{ // Received full stop command
					printf("Received STOP command!\n");
					break;
				}
				else if (cmdID == 'P' && uart_fetchCmd())
				{ // Received ping, answer ping
					// No need to respond to ping, blob reports are enough
					lastPing = std::chrono::high_resolution_clock::now();
				}
			}

			// Wait for CPU thread to update blobs
			blobsDone.lock();

			// Signal visualization thread that it can access blobs
			if (displayViz) vizReady.unlock();

			if (useUART)
			{
				// Create blob report
				int blobReportLength = blobs.size()*6 + 4;
				blobReportBuffer.resize(blobReportLength);
				blobReportBuffer[0] = '#';
				blobReportBuffer[1] = 'B';
				blobReportBuffer[2] = hex[(blobs.size()*6) >> 4];
				blobReportBuffer[3] = hex[(blobs.size()*6) & 15];

				// Fill with blob data
				uint16_t *blobData = (uint16_t*)&blobReportBuffer[4];
				for (int i = 0; i < blobs.size(); i++)
				{
					Cluster *blob = &blobs[i];
					uint8_t blobColor = 0b1111;
					int blobX = (int)((double)blob->centroid.X * 65536 / params.width);
					int blobY = (int)((double)blob->centroid.Y * 65536 / params.height);
					int blobS = std::min((int)(blob->centroid.S * 2), 255);
					blobData[i*3+0] = blobX;
					blobData[i*3+1] = blobY;
					blobData[i*3+2] = (blobS & 0xFF) | ((blobColor & 0xFF) << 8);
					/*int blobX = (int)((double)blob->centroid.X * 16383 / params.width);
					int blobY = (int)((double)blob->centroid.Y * 16383 / params.height);
					int blobS = std::min((int)(blob->centroid.S * 2), 255);
					uint32_t data = (blobX << 0) | (blobY << 14)
							|  << 22
							| (blobColor & ((1<<4)-1)) << 28;
					blobData[i] = data;
					*/
				}

				// Send blob report over UART
				write(uartFD, &blobReportBuffer[0], blobReportLength);
			}

//			if (!displayViz)
//				printf("Blobs: %d\n", blobs.size());

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
			if (numFrames % 100 == 0)
			{
				// Check timeout
				auto currentTime = std::chrono::high_resolution_clock::now();
				int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPing).count();
				if (elapsedMS > UART_PING_TIMEOUT)
				{ // Setup timeout, send NAK
					printf("Ping timeout exceeded, resetting comm!\n");
					write(uartFD, msg_nak, sizeof(msg_nak)-1);
					tcdrain(uartFD); // Wait for bytes to be sent
					uart_reset();
					break;
				}

				// Check input
				char cin;
				if (read(STDIN_FILENO, &cin, 1) == 1)
				{
					if (iscntrl(cin)) printf("%d", cin);
					else if (cin == 'q')
					{ // Complete stop of program requested
						running = false;
						break;
					}
					else printf("%c", cin);
				}
			}

			// Wait for visualization thread to finish accessing blobs
			if (displayViz) vizDone.lock();

			// Signal CPU thread that blobs can be updated again
			blobsReady.unlock();
		}

		printf("Stopping blob detection!\n");

		// Make sure all threads conclude
		blobDetectionRunning = false;
		blobmaskReady.unlock();
		blobmaskDone.unlock();
		blobsReady.unlock();
		vizReady.unlock();
		GPUThread.join();
		CPUThread.join();
		if (displayViz && VizThread.joinable()) VizThread.join();

		if (running && rsp_id && rsp_ack) goto phase_setup;
		else if (running) goto phase_identification;

abort:
		if (uartFD >= 0) close(uartFD);

	}


	printf("------------------------\n");

	return 0;
}

static void GPUBlobDetectionThread()
{
	int status;
	CamGL_Frame *frame;

	// Setup EGL context
	if (setupEGLPBuffer(&eglSetup) != 0)
	{
		printf("Failed to setup EGL pbuffer context!\n");
		goto finish;
	}

	// Init camera GL
	camGL = camGL_create(eglSetup, &params);
	if (camGL == NULL)
	{
		printf("Failed to create Camera GL!\n");
		goto egl;
	}

	// Let camera initialize
	sleep(1);

	// Setup GL resources
	if (!initBlobDetection(params.width, params.height, eglSetup))
	{
		printf("Failed to init blob detection!\n");
		goto camgl;
	}

	// Start camera capturing
	status = camGL_startCamera(camGL);
	if (status != CAMGL_SUCCESS)
	{
		printf("Failed to start camera GL with code %d!\n", status);
		goto blob;
	}

	// Get handle to frame struct, stays the same when frames are updated
	frame = camGL_getFrame(camGL);
	while ((status = camGL_nextFrame(camGL)) == CAMGL_SUCCESS && blobDetectionRunning)
	{ // Frames was available and has been processed

		if (!blobDetectionRunning) goto blob;

		// Render blob mask into GPU texture
		performBlobDetectionGPU(frame);

		// Wait for CPU thread to finish accessing last blob mask
		blobmaskReady.lock();

		// Copy regions with blobs into CPU blob mask
		performBlobDetectionRegionsFetch();

		// Signal CPU thread that it can access the next blob mask
		blobmaskDone.unlock();
	}

	if (status != 0)
		printf("Camera GL was interrupted with code %d!\n", status);
	else
		camGL_stopCamera(camGL);

blob:
	cleanBlobDetection();
camgl:
	camGL_destroy(camGL);
egl:
	terminateEGL(&eglSetup);
	eglSetup.context = EGL_NO_CONTEXT;
finish:
	blobDetectionRunning = false;
	blobmaskReady.unlock();
	blobmaskDone.unlock();
	vizReady.unlock();
	blobsReady.unlock();
	blobsDone.unlock();
}

static void CPUBlobDetectionThread()
{
	while (blobDetectionRunning)
	{
		// Wait for GPU thread to create blob mask
		blobmaskDone.lock();

		if (!blobDetectionRunning) break;

		// Perform blob detection on blob mask
		blobsWIP.clear();
		performBlobDetectionCPU(blobsWIP);

		// Signal GPU thread that it can update the blob mask
		blobmaskReady.unlock();

		// Wait for main thread to finish read access on last frames blobs
		blobsReady.lock();

		// Update blobs
		blobs.swap(blobsWIP);

		// Signal main thread that blobs have been updated
		blobsDone.unlock();
	}

	blobsDone.unlock();
}

static void VisualizationThread()
{
	int dispWidth, dispHeight;
	float renderRatioCorrection;

	// Wait for GPU thread to setup main EGL context
	while (blobDetectionRunning && displayViz && eglSetup.context == EGL_NO_CONTEXT);
	if (!blobDetectionRunning || !displayViz) return;

	// Create native window (not real GUI window) to render to
	EGL_Window window;
	if (createWindow(&window) != 0)
	{
		printf("Failed to setup native window!\n");
		displayViz = false;
		goto finish;
	}
	dispWidth = window.eglWindow.width;
	dispHeight = window.eglWindow.height;
	renderRatioCorrection = (((float)dispHeight / params.height) * params.width) / dispWidth;

	// Setup EGL context
	if (setupEGLWindow(&eglDisplaySetup, &window, &eglSetup) != 0)
	{
		printf("Failed to setup EGL window context!\n");
		displayViz = false;
		goto window;
	}
	glClearColor(0.8f, 0.2f, 0.1f, 1.0f);

	while (blobDetectionRunning && displayViz)
	{
		// Wait for main thread to allow access to blobs
		vizReady.lock();

		if (!blobDetectionRunning || !displayViz) break;

		// Visualization view bounds
	#ifdef BLOB_VIZ_FOCUS
		Bounds viewBounds { params.width, params.height, 0, 0 };
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
		Bounds viewBounds { 0, 0, params.width, params.height };
	#endif

		// Visualization
		glViewport((int)((1-renderRatioCorrection) * dispWidth / 2), 0, (int)(renderRatioCorrection * dispWidth), dispHeight);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		// Visualize found points
		visualizeBlobDetection(blobs, viewBounds, (float)(viewBounds.maxX-viewBounds.minX)/dispWidth);
		// Submit frame
		eglSwapBuffers(eglDisplaySetup.display, eglDisplaySetup.surface);

		// Signal main thread that visualization is done accessing blobs
		vizDone.unlock();
	}

egl:
	terminateEGL(&eglDisplaySetup);
window:
	destroyWindow(&window);
finish:
	vizDone.unlock();
}

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode()
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &terminalSettings);
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
