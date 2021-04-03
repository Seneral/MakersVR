/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <math.h>
// Console and UART
#include <termios.h>
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR

#include "fbUtil.h"
#include "qpu/qpu_program.h"
#include "qpu/qpu_info.h"
#include "camera/gcs.h"
#include "blobdetection.hpp"

#include "interface/mmal/mmal_encodings.h"
#include "bcm_host.h"
#include "interface/vcsm/user-vcsm.h" // for vcsm_vc_hdl_from_ptr

#define RUN_CAMERA	// Have the camera supply frames

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

	GCS_CameraParams params = {
		.mmalEnc = MMAL_ENCODING_I420,
		.width = 1640,
		.height = 1232,
		.fps = 30,
		.shutterSpeed = 500,
		.iso = 0,
		.disableEXP = true,
		.digitalGain = 1, // Only in effect when exp explicitly disabled, max 16
		.analogGain = 2, // Only in effect when exp explicitly disabled, max 16
		.disableAWB = true,
		.disableISPBlocks = 0 // https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=175711
//			| (1<<2) // Black Level Compensation
			| (1<<3) // Lens Shading
			| (1<<5) // White Balance Gain
			| (1<<7) // Defective Pixel Correction
			| (1<<9) // Crosstalk
			| (1<<18) // Gamma
			| (1<<22) // Sharpening
//			| (1<<24) // Some Color Conversion
	};
	const int blockLength = 20;
	char codeFile[64];
	const char *copyFile = "qpu_copy.bin";
	int padding = 2;
	bool enableQPU[12] = { 1,1,1,0, 1,1,0,1, 1,0,1,0 };
	int thresholdCO = 255*0.04;
	int diffCO = 255*0.025;
	bool zeroCompat = false;
	bool debugViz = false;
	bool useUART = false;

	int arg;
	while ((arg = getopt(argc, argv, "c:w:h:f:s:g:m:n:q:pzdu")) != -1)
	{
		switch (arg)
		{
			case 'c':
				{
					int size = strnlen(optarg, sizeof(codeFile));
					memcpy(codeFile, optarg, size);
					codeFile[size] = '\0';
				}
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
			case 's':
				params.shutterSpeed = std::stoi(optarg);
				break;
			case 'g':
				params.analogGain = std::stoi(optarg);
				break;
			case 'm':
				thresholdCO = 255*std::stof(optarg);
				break;
			case 'n':
				diffCO = 255*std::stof(optarg);
				break;
			case 'q':
				for (int i = 0; i < 12 && i < strlen(optarg); i++)
					enableQPU[i] = optarg[i] == '1';
				break;
			case 'p':
				padding = 0;
				break;
			case 'z':
				zeroCompat = true;
				break;
			case 'd':
				debugViz = true;
				break;
			case 'u':
				useUART = true;
				break;
			default:
				printf("Usage: %s -c codefile [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-g analog gain {0,16}] [-m main threshold 0-1] [-n diff threshold 0-1] [-q QPU mask {0,1}^12] [-p disable padding] [-z Zero compat mode] [-d debug visualization]\n", argv[0]);
				return -1;
		}
	}
	if (optind < argc - 1)
	{
		printf("Usage: %s -c codefile [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-g analog gain {0,16}] [-m main threshold 0-1] [-n diff threshold 0-1] [-q QPU mask {0,1}^12] [-p disable padding] [-z Zero compat mode] [-d debug visualization]\n", argv[0]);
		return -1;
	}

	// ---- Checks ----
	if (FILE *file = fopen(codeFile, "r"))
		fclose(file);
	else
	{
		printf("Main code file %s does not exist!\n", codeFile);
		return -1;
	}
	if (FILE *file = fopen(copyFile, "r"))
		fclose(file);
	else
	{
		printf("QPU copy code file %s does not exist!\n", copyFile);
		return -1;
	}

	// ---- Init ----

	// Init BCM Host
	bcm_host_init();

	// ---- Setup framebuffer ----

	int fbfd = 0;
	struct fb_var_screeninfo orig_vinfo;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	if (debugViz)
	{ // Get frame buffer information
		fbfd = setupFrameBuffer(&orig_vinfo, &vinfo, &finfo, true);
		if (!fbfd) debugViz = false;
	}

	// ---- Start Loop ----

	// For non-blocking input even over ssh
	setConsoleRawMode();

	bool running = true, error = false;
	int uartFD = -1;
	// Frame Counter
	std::chrono::high_resolution_clock::time_point startTime, lastTime, lastPing;
	int lastErrorFrames = 0, numFrames = 0, lastFrames = 0;

abort:
	while (running)
	{
		// Go to appropriate place depending on the state we left of with
		if (uartFD >= 0)
		{ // Go handle UART connection
			if (rsp_id && rsp_ack) goto phase_setup;
			else goto phase_identification;
		}

		// ---- Init UART Communication ----

		// Open UART port
		//if (uartFD >= 0) close(uartFD);
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
			uartFD = -1;
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
					params.analogGain = *(uint8_t*)&uartRcv[cmdPos+8];
					thresholdCO = *(uint8_t*)&uartRcv[cmdPos+9];
					diffCO = *(uint8_t*)&uartRcv[cmdPos+10];
					printf("Received setup: %dx%d @ %d fps, %dns shutter speed, %dx gain, m=%.2f, n=%.2f \n", params.width, params.height, params.fps, params.shutterSpeed, params.analogGain, thresholdCO/255.0f, diffCO/255.0f);
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


		// ---- Init ----

		// Core QPU structures
		QPU_BASE base;
		QPU_PROGRAM blobProgram;
		QPU_PROGRAM copyProgram;
		QPU_BUFFER bitmskBuffer;
		QPU_BUFFER zeroCpyBuffer;
		// QPU Debugging
		QPU_PerformanceState perfState;
		QPU_HWConfiguration hwConfig;
		QPU_UserProgramInfo upInfo;
		// MMAL Camera
#ifdef RUN_CAMERA
		GCS *gcs;
#endif
		// Performance Measurement
		double cameraTimeAvg = 0;
		double qpuTimeAvg = 0;
		double fetchTimeAvg = 0;
		double cclTimeAvg = 0;
		const int avgFloating = 50;
#ifndef RUN_CAMERA
		// Camera emulation buffers
		const int emulBufCnt = 4;
		QPU_BUFFER camEmulBuf[emulBufCnt];
#endif
		// QPU usage
		int qpusUsed = 0;
		for (int i = 0; i < 12; i++)
			qpusUsed += enableQPU[i]? 1 : 0;
		// Buffers
		std::vector<Cluster> blobs;
		std::vector<Cluster> blobsOld;
		std::vector<uint8_t> blobReportBuffer;

		// Init QPU Base (basic information to work with QPU)
		int ret = qpu_initBase(&base);
		if (ret != 0)
		{
			printf("Failed to init qpu base! %d \n", ret);
			return ret;
		}


		// ---- Bitmask setup ----

		// MMAL frame buffers are always multiples of 32x16
		uint32_t srcStride = params.width/32*32;
		if (params.width > srcStride) srcStride += 32;
		// Width and height must be multiple of 8*16 and blockLength respectively
		uint32_t lineWidth = (params.width-padding*2)/8/16*8*16;
		uint32_t lineCount = params.height/blockLength*blockLength;

		// Set up bit target, one bit per pixel
		qpu_allocBuffer(&bitmskBuffer, &base, lineWidth/8*lineCount, 4096);
		uint32_t tgtStride = lineWidth/8;
		uint32_t tgtBufferPtr = bitmskBuffer.ptr.vc;

		// Setup intermediary buffer for zero compability
		if (zeroCompat)
			qpu_allocBuffer(&zeroCpyBuffer, &base, srcStride*params.height, 4096);


		// ---- Generate tiling setup ----

		// Split image in columns of 8x16 pixels assigned to one QPU each.
		// Split vertically until most or all QPUs are used
		int numTileCols = lineWidth/8; // Num of 8px tiles in a row
		int numTileRows = lineCount/blockLength; // Num of tiles in a col
		int numProgCols = numTileCols/16; // Number of instances required (QPU is 16-way)
		int splitCols = 1;
		while (numProgCols * (splitCols+1) <= qpusUsed)
		{ // Split columns among QPUs to minimize the number of idle QPUs
			splitCols++;
		}
		int numInstances = numProgCols * splitCols;
		int droppedPixels = params.width - numProgCols*16*8; // Some are dropped for maximum performance, extra effort is not worth it
		int hOffset = (droppedPixels/2-padding)/4*4; // Make an attempt to center the image, but still align to 4 bytes
		printf("SETUP: %d instances processing 1/%d columns each, covering %dx%d tiles, plus %d horizontal pixels dropped, offset %d\n",
			numInstances, splitCols, numProgCols*16, lineCount/4/splitCols, droppedPixels, hOffset);


		// ---- Setup program ----

		// Setup program with specified progmem sizes
		const int numUnif = 7;
		QPU_PROGMEM progmemSetup {
			.codeSize = qpu_getCodeSize(codeFile), //4096*4;
			.uniformsSize = (uint32_t)numInstances*numUnif,
			.messageSize = 2
		};
		qpu_initProgram(&blobProgram, &base, progmemSetup);
		qpu_loadProgramCode(&blobProgram, codeFile);

		// Set up uniforms of the QPU program
		qpu_lockBuffer(&blobProgram.progmem_buffer);
		for (int c = 0; c < numProgCols; c++)
		{
			for (int r = 0; r < splitCols; r++)
			{ // Set up each program instance with their column
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 0] = 0; // Enter source pointer each frame
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 1] = tgtBufferPtr + c*16*blockLength + r*numTileRows/splitCols*blockLength*tgtStride;
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 2] = srcStride;
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 3] = tgtStride;
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 4] = numTileRows/splitCols;
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 5] = thresholdCO;
				blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 6] = diffCO;
			}
		}
		qpu_unlockBuffer(&blobProgram.progmem_buffer);


		// ---- Setup Copy Program ----

		if (zeroCompat)
		{
			// Setup program with specified progmem sizes
			const int numCpyUnif = 3;
			QPU_PROGMEM copyProgmemSetup {
				.codeSize = qpu_getCodeSize(copyFile), //4096*4;
				.uniformsSize = (uint32_t)numCpyUnif,
				.messageSize = 0
			};
			qpu_initProgram(&copyProgram, &base, copyProgmemSetup);
			qpu_loadProgramCode(&copyProgram, copyFile);

			// Set up uniforms of the QPU program
			qpu_lockBuffer(&copyProgram.progmem_buffer);
			copyProgram.progmem.uniforms.arm.uptr[0] = 0;
			copyProgram.progmem.uniforms.arm.uptr[1] = zeroCpyBuffer.ptr.vc;
			copyProgram.progmem.uniforms.arm.uptr[2] = srcStride*params.height/64;
			qpu_unlockBuffer(&copyProgram.progmem_buffer);
		}


		// ---- Setup QPU ----

		// Enable QPU
		if (qpu_enable(base.mb, 1)) {
			printf("QPU enable failed!\n");
			error = true;
			goto error_qpu;
		}
		printf("-- QPU Enabled --\n");

		// Debug QPU Hardware
		qpu_debugHW(&base);
		// VPM memory reservation
		base.peripherals[V3D_VPMBASE] = 16; // times 4 to get number of vectors; Default: 8 (32/4), Max: 16 (64/4)
		qpu_getHWConfiguration(&hwConfig, &base);
		qpu_getUserProgramInfo(&upInfo, &base);
		printf("Reserved %d / %d vectors of VPM memory for user programs!\n", upInfo.VPMURSV_V, hwConfig.VPMSZ_V);
		// QPU scheduler reservation
	//	for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter
	//		qpu_setReservationSetting(&base, i, enableQPU[i]? 0b1110 : 0b1111);
		for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter
			qpu_setReservationSetting(&base, i, enableQPU[i]? 0b1110 : 0b0001);
		qpu_logReservationSettings(&base);
		// Setup performance monitoring
		qpu_setupPerformanceCounters(&base, &perfState);
		perfState.qpusUsed = std::min(numInstances, qpusUsed);


		// ---- Setup Blob Detection ----

		initBlobDetection(lineWidth, lineCount, hOffset+padding);


		// ---- Setup Camera ----

		// Create GPU camera stream (MMAL camera)
	#ifdef RUN_CAMERA
		gcs = gcs_create(&params);
		if (gcs == NULL)
		{
			printf("Failed to greate GCS! \n");
			error = true;
			goto error_gcs;
		}
		gcs_start(gcs);
		printf("-- Camera Stream started --\n");
	#else

		// Generate random test frames
		for (int i = 0; i < emulBufCnt; i++)
		{ // Allocate only grayscale buffer
			qpu_allocBuffer(&camEmulBuf[i], &base, srcStride*params.height, 4096);
		}
		srand((unsigned int)time(NULL));
		for (int i = 0; i < emulBufCnt; i++)
		{
			qpu_lockBuffer(&camEmulBuf[i]);
			uint8_t *YUVFrameData = (uint8_t*)camEmulBuf[i].ptr.arm.vptr;
			for (int y = 0; y < params.height; y++)
				for (int x = 0; x < params.width; x++)
					YUVFrameData[srcStride*params.height*0 + y*srcStride + x] = 0;
			// Write test blobs
			for (int c = 0; c < 5; c++)
			{
				float size = 40.0f;
				int posX = params.width/2-50+100*(i&1);
				int posY = params.height/2-50+100*((i&2)>>1);
	//			float size = (rand()%255) / 255.0f * 5.0f + 1.0f;
	//			int posX = rand()%params.width;
	//			int posY = rand()%params.height;
				int dots = 0;
				for (int y = -size; y < size; y++)
				for (int x = -size; x < size; x++)
				{
					if (x*x + y*y <= size*size)
					{
						int pX = posX+x, pY = posY+y;
						if (pX >= 0 && pY >= 0 && pX < params.width && pY < params.height)
						{
							YUVFrameData[pY*srcStride + pX] = 255;
							dots++;
						}
					}
				}
				printf("Blob at %d, %d has %d dots!\n", posX, posY, dots);
			}
			qpu_unlockBuffer(&camEmulBuf[i]);
		}
	#endif

		// Statistics
		startTime = std::chrono::high_resolution_clock::now();
		lastTime = startTime;
		numFrames = 0;
		lastFrames = 0;

		while (true)
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

#ifdef RUN_CAMERA
			// Get most recent MMAL buffer from camera
			void *cameraBufferHeader = gcs_requestFrameBuffer(gcs);
			if (!cameraBufferHeader)
			{
				printf("GCS returned NULL frame! \n");
				error = true;
				break;
			}
			else
#else
			// Emulate framerate
			usleep(std::max(0,(int)(1.0f/params.fps*1000*1000)-4000));
#endif
			{
				clock_t perfClock = clock();

				// ---- Camera Frame Access ----

				uint32_t cameraBufferPtr;
	#ifdef RUN_CAMERA
				// Get buffer data from opaque buffer handle
				void *cameraBuffer = gcs_getFrameBufferData(cameraBufferHeader);
				uint32_t cameraBufferHandle = 0;

				// Get VC-space buffer address of original buffer
				// Source: https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=167652
				// Get VCSM Handle of frameBuffer (works only if zero-copy is enabled, so buffer is in VCSM)
				cameraBufferHandle = vcsm_vc_hdl_from_ptr(cameraBuffer);
				// Lock VCSM buffer to get VC-space address
				cameraBufferPtr = mem_lock(base.mb, cameraBufferHandle);
	#else
				// Use prepared testing frames
				qpu_lockBuffer(&camEmulBuf[numFrames%emulBufCnt]);
				cameraBufferPtr = camEmulBuf[numFrames%emulBufCnt].ptr.vc;
	#endif

				if (zeroCompat)
				{ // Manually copy camera frame to custom VCSM buffer

// Standard CPU based copy is around 5 times slower than QPU based DMA copy
//					qpu_lockBuffer(&zeroCpyBuffer);
//					memcpy(zeroCpyBuffer.ptr.arm.vptr, cameraBuffer, srcStride*params.height);
//					cameraBufferPtr = zeroCpyBuffer.ptr.vc;

					// Set source frame address
					qpu_lockBuffer(&copyProgram.progmem_buffer);
					copyProgram.progmem.uniforms.arm.uptr[0] = cameraBufferPtr;
					qpu_unlockBuffer(&copyProgram.progmem_buffer);

					// Lock target buffer
					qpu_lockBuffer(&zeroCpyBuffer);
					// Execute one program with mailbox (code and uniform have been setup in messages on init)
					int result = qpu_executeProgram(&copyProgram, &base, 1);
					// Log errors occurred during execution
					qpu_logErrors(&base);

					if (result != 0)
					{
						printf("Encountered an error during copying after %d frames!\n", numFrames);
						error = true;
						break;
					}

					// Update source buffer to use
					cameraBufferPtr = zeroCpyBuffer.ptr.vc;
				}

				// Performance Diagnostics
				cameraTimeAvg += ((double)(clock() - perfClock)/CLOCKS_PER_SEC*1000-cameraTimeAvg) / std::min(numFrames+1, avgFloating);
				perfClock = clock();

				// ---- Uniform preparation ----

				// Set source buffer pointer in progmem uniforms
				qpu_lockBuffer(&blobProgram.progmem_buffer);
				// Set up individual source pointer for each program instance
				for (int c = 0; c < numProgCols; c++)
					for (int r = 0; r < splitCols; r++)
						blobProgram.progmem.uniforms.arm.uptr[(c*splitCols+r)*numUnif + 0] = cameraBufferPtr + c*8*16 + r*lineCount/splitCols*srcStride + hOffset;
				qpu_unlockBuffer(&blobProgram.progmem_buffer);

				// ---- Program execution ----

				// Lock target buffer
				qpu_lockBuffer(&bitmskBuffer);

				// Execute numInstances programs each with their own set of uniforms
				int result = qpu_executeProgramDirect(&blobProgram, &base, numInstances, numUnif, numUnif, &perfState);

				// Log errors occurred during execution
				qpu_logErrors(&base);

				if (zeroCompat) // Unlock intermediate buffer
					qpu_unlockBuffer(&zeroCpyBuffer);

	#ifdef RUN_CAMERA
				// Unlock VCSM buffer
				mem_unlock(base.mb, cameraBufferHandle);

				// Return camera buffer to camera
				gcs_returnFrameBuffer(gcs);
	#else
				qpu_unlockBuffer(&camEmulBuf[numFrames%emulBufCnt]);
	#endif

				if (result != 0)
				{
					printf("Encountered an error after %d frames!\n", numFrames);
					error = true;
					break;
				}

				// Performance Diagnostics
				qpuTimeAvg += ((double)(clock() - perfClock)/CLOCKS_PER_SEC*1000-qpuTimeAvg) / std::min(numFrames+1, avgFloating);
				perfClock = clock();

				// Extract regions with blobs
				performBlobDetectionRegionsFetch(bitmskBuffer.ptr.arm.uptr);

				// Performance Diagnostics
				fetchTimeAvg += ((double)(clock() - perfClock)/CLOCKS_PER_SEC*1000-fetchTimeAvg) / std::min(numFrames+1, avgFloating);
				perfClock = clock();

				// Perform Connected Component Labelling on regions
				blobs.clear();
				performBlobDetectionCPU(blobs);
//				printf("Found %d Blobs!\n", (int)blobs.size());
//				for (int i = 0; i < blobs.size(); i++)
//					printf("Blob %d has %d blobs!\n", i, (int)blobs[i].dots.size());

				// Performance Diagnostics
				cclTimeAvg += ((double)(clock() - perfClock)/CLOCKS_PER_SEC*1000-cclTimeAvg) / std::min(numFrames+1, avgFloating);

				// Unlock target buffer
				qpu_unlockBuffer(&bitmskBuffer);

				// ---- Send UART Packet ----

				if (useUART)
				{
					// Create blob report
					int blobPayloadLength = blobs.size()*6;
					int blobReportLength = blobPayloadLength + 4;
					blobReportBuffer.resize(blobReportLength);
					blobReportBuffer[0] = '#';
					blobReportBuffer[1] = 'B';
					blobReportBuffer[2] = hex[blobPayloadLength >> 4];
					blobReportBuffer[3] = hex[blobPayloadLength & 15];

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
				//		int blobX = (int)((double)blob->centroid.X * 16383 / params.width);
				//		int blobY = (int)((double)blob->centroid.Y * 16383 / params.height);
				//		int blobS = std::min((int)(blob->centroid.S * 2), 255);
				//		uint32_t data = (blobX << 0) | (blobY << 14)
				//				|  << 22
				//				| (blobColor & ((1<<4)-1)) << 28;
				//		blobData[i] = data;
					}

					// Send blob report over UART
					write(uartFD, &blobReportBuffer[0], blobReportLength);
				}


				// ---- Performance and Statistics ----

				numFrames++;
				if (numFrames % 100 == 0)
				{ // Frames per second
					auto currentTime = std::chrono::high_resolution_clock::now();
					int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
					float elapsedS = (float)elapsedMS / 1000;
					lastTime = currentTime;
					int frames = (numFrames - lastFrames);
					lastFrames = numFrames;
					float fps = frames / elapsedS;
					printf("%d frames over %.2fs (%.1ffps); Frame Copy: %.2fms, QPU: %.2fms; Fetch: %.2fms; CCL: %.2fms! \n", frames, elapsedS, fps, cameraTimeAvg, qpuTimeAvg, fetchTimeAvg, cclTimeAvg);
				}
				if (numFrames % 10 == 0)
				{ // Detailed QPU performance gathering (every 10th frame to handle QPU performance register overflows)
					qpu_updatePerformance(&base, &perfState);
				}
				if (numFrames % 100 == 0 && numFrames <= 500)
				{ // Detailed QPU performance report
					printf("QPU Performance over past 500 frames:\n");
					qpu_logPerformance(&perfState);
				}
				if (numFrames % 500 == 0 && numFrames > 500)
				{ // Detailed QPU performance report
					printf("QPU Performance over past 500 frames:\n");
					qpu_logPerformance(&perfState);
				}
				if (numFrames % 100 == 0)
				{
					// Check timeout
					auto currentTime = std::chrono::high_resolution_clock::now();
					int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPing).count();
					if (uartFD >= 0 && elapsedMS > UART_PING_TIMEOUT)
					{ // Setup timeout, send NAK
						printf("Ping timeout exceeded, resetting comm!\n");
						write(uartFD, msg_nak, sizeof(msg_nak)-1);
						tcdrain(uartFD); // Wait for bytes to be sent
						uart_reset();
						break;
					}
				}
				if (numFrames % 10 == 0)
				{
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
						else if (cin == 'r')
						{ // Reset/Restart, stop current blob detection and start again
							break;
						}
						else printf("%c", cin);
					}
				}

				// ---- Framebuffer debugging ----

				int ilCnt = (int)std::ceil((float)params.fps/30);
				if (debugViz && (numFrames % ilCnt) == 0)
				{ // Manual access to framebuffer
					void *fbp = lock_fb(fbfd, finfo.smem_len);
					if ((int)fbp == -1)
						printf("Failed to mmap framebuffer!\n");
					else
					{ // Blit debug visualizations to frame buffer

						// Determine visualization resolution and mapping (fit into maximum window)
						const int vizH = 640, vizV = 480;
						int dH = std::min((int)params.width, vizH), dV = std::min((int)params.height, vizV);
						dH = std::min(dV*params.width/params.height, vizH);
						dV = std::min(dH*params.height/params.width, vizV);

						// Blit raw blob mask
	/*
						qpu_lockBuffer(&bitmskBuffer);
						for (int y = 0; y < dV; y++)
						{
	//						int tgtY = y * lineCount / dV;
							int tgtY = y + (params.height/2-dV/2);
							uint8_t *px = (uint8_t*)fbp + y*finfo.line_length;
							uint8_t *maskBase = (uint8_t*)bitmskBuffer.ptr.arm.uptr;
							int blkFactor = blockLength;
							int blkLine = tgtY % blockLength;
							maskBase += (tgtY-blkLine) * tgtStride;
							int intLine = tgtY%4;
							int blkInt = blkLine - intLine;
							for (int x = 0; x < dH; x++)
							{
	//							int tgtX = x * lineWidth / dH;
								int tgtX = x + (params.width/2-dH/2);
								uint8_t *mask = maskBase + tgtX / 8 * blkFactor + blkInt + tgtX % 4;
								*(uint32_t*)px = ((*mask >> ((tgtX%8)/4+intLine*2)) & 1)? 0xFFFFFFFF : 0xFF000000;
								px += vinfo.bits_per_pixel/8;
							}
						}
						qpu_unlockBuffer(&bitmskBuffer);
	*/

						// Start with black canvas
						if (numFrames == ilCnt)
						{
							for (int i = 0; i < dV; i++)
							{
								uint32_t *ptr = (uint32_t*)fbp + i * finfo.line_length/4;
								uint32_t *max = ptr + dH;
								for (; ptr < max; ptr++)
									*ptr = 0xFF000000;
							}
						}

						// Erase old blobs
						for (int c = 0; c < blobsOld.size(); c++)
						{
							Cluster *cluster = &blobsOld[c];
							for (int p = 0; p < cluster->dots.size(); p++)
							{
								int x = cluster->dots[p].X, y = cluster->dots[p].Y;
								int tgtX = x * dH / params.width, tgtY = y * dV / params.height;
	//							int tgtX = x - (params.width/2-dH/2), tgtY = y - (params.height/2-dV/2);
	//							if (tgtX < 0 || tgtX >= dH || tgtY < 0 || tgtY >= dV) continue;
								uint8_t *px = (uint8_t*)fbp + tgtY*finfo.line_length + tgtX*4;
								*(uint32_t*)px = 0xFF000000;
							}
						}

						// Write new blobs
						for (int c = 0; c < blobs.size(); c++)
						{
							Cluster *cluster = &blobs[c];
							for (int p = 0; p < cluster->dots.size(); p++)
							{
								int x = cluster->dots[p].X, y = cluster->dots[p].Y;
								int tgtX = x * dH / params.width, tgtY = y * dV / params.height;
	//							int tgtX = x - (params.width/2-dH/2), tgtY = y - (params.height/2-dV/2);
	//							if (tgtX < 0 || tgtX >= dH || tgtY < 0 || tgtY >= dV) continue;
								uint8_t *px = (uint8_t*)fbp + tgtY*finfo.line_length + tgtX*4;
								*(uint32_t*)px = 0xFF000000 + (c&1? 0x0000FF00 : 0x00FF0000);
							}
						}

						blobsOld.swap(blobs);

						unlock_fb(fbp, finfo.smem_len);
					}
/*
					if (blobsOld.size() > 1)
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(1000));
						char cin;
						while (read(STDIN_FILENO, &cin, 1) == 1 && cin ==  'h')
							std::this_thread::sleep_for(std::chrono::milliseconds(5000));
					}
*/
				}
			}
		}

		printf("Stopping blob detection!\n");

	#ifdef RUN_CAMERA
		gcs_stop(gcs);
		gcs_destroy(gcs);
		printf("-- Camera Stream stopped --\n");

	#else
		for (int i = 0; i < emulBufCnt; i++)
			qpu_releaseBuffer(&camEmulBuf[i]);
	#endif


error_gcs:

		// Disable QPU
		if (qpu_enable(base.mb, 0))
			printf("-- QPU Disable failed --\n");
		else
			printf("-- QPU Disabled --\n");

		for (int i = 0; i < 12; i++) // Reset all QPUs to be freely sheduled
			qpu_setReservationSetting(&base, i, 0b0000);

error_qpu:

		if (zeroCompat)
			qpu_releaseBuffer(&zeroCpyBuffer);

		qpu_releaseBuffer(&bitmskBuffer);

		qpu_destroyProgram(&blobProgram);

		if (zeroCompat)
			qpu_destroyProgram(&copyProgram);

		qpu_destroyBase(&base);

		if (error && useUART)
		{ // TODO: Change to stopStreamingRequest, instead of discarding all comm
			write(uartFD, msg_nak, sizeof(msg_nak)-1);
			tcdrain(uartFD); // Wait for bytes to be sent
			uart_reset();
			if (numFrames-lastErrorFrames < 10)
				running = false; // Restart service
			lastErrorFrames = numFrames;
		}
		else if (error)
		{
			running = false;
		}

	}

	if (debugViz)
		close(fbfd);

	printf("------------------------\n");

	return EXIT_SUCCESS;
}

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode()
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{ // Reset at exit
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
	struct termios tty = {};

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
