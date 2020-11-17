/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "configurator.hpp"

#include "util.hpp" // StatValue
#include "testing.hpp"
#include "calibration.hpp"
#include "tracking.hpp"

#include "wxbase.hpp" // wxLog*

#include <iomanip>
#include <chrono>
#include <bitset>

#define MEASURE_RECEIVE_RATE
//#define DEBUG_BLOBS_IN
//#define DEBUG_TRANSFER_IN

/* Functions */

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length, void *userData);
static void onIsochronousIN(uint8_t *data, int length, void *userData);
static void onInterruptIN(uint8_t *data, int length, void *userData);

static void CommThread(ConfiguratorState *state);
static void TestThread(ConfiguratorState *state);

inline void printBuffer(std::stringstream &ss, uint8_t *buffer, uint8_t size)
{
	ss << "0x" << std::uppercase << std::hex;
	for (int i = 0; i < size; i++)
		ss << std::setfill('0') << std::setw(2) << (int)buffer[i];
}

/* Testing and Debug variables */

#ifdef MEASURE_RECEIVE_RATE
static std::chrono::time_point<std::chrono::steady_clock> g_lastReceiveTime;
static StatValue receiveRate;
static std::atomic<long long> g_receiveCount = 0;
#endif

const int usbHeaderSize = 8;

ConfiguratorState *globalState;


bool ConfiguratorInit(ConfiguratorState &state)
{
	globalState = &state;

	// Read config
	parseConfigFile("config/config.json", &state.config);

	// Get built-in calibration markers
	getBuiltInMarkers(state.control.calibration.markerTemplates2D);

	// Copy calibration testing markers into database
	int prevCalibCnt = state.control.calibration.markerTemplates2D.size();
	state.control.calibration.markerTemplates2D.reserve(prevCalibCnt + state.config.testing.calibrationMarkers.size());
	for (int i = 0; i < state.config.testing.calibrationMarkers.size(); i++)
	{
		state.control.calibration.markerTemplates2D.push_back(state.config.testing.calibrationMarkers[i]);
		state.control.calibration.markerTemplates2D[prevCalibCnt+i].id = prevCalibCnt+i;
	}
	if (state.control.calibration.markerTemplates2D.size() > 0)
		state.control.calibration.markerTemplate2D = &state.control.calibration.markerTemplates2D[0];

	// Assign IDs to tracking testing markers (trying to be consistent)
	for (int i = 0; i < state.config.testing.trackingMarkers.size(); i++)
		state.config.testing.trackingMarkers[i].id = -i-1;
	if (state.config.testing.trackingMarkers.size() > 0)
	{
		state.control.testing.markerTemplate3D = &state.config.testing.trackingMarkers[0];
		state.control.tracking.trackID = state.control.testing.markerTemplate3D->id; // TODO: Detect marker ID
	}

	return true;
}

void ConfiguratorExit(ConfiguratorState &state)
{
	// Clean up usb device communication
	comm_exit(&state.comm);
}

bool CommSetup(ConfiguratorState &state)
{
	// Setup usb device communication
	if (!comm_init(&state.comm)) return false;
	state.comm.userData = &state;
	state.comm.onControlResponse = onControlResponse;
	state.comm.onInterruptIN = onInterruptIN;
	state.comm.onIsochronousIN = onIsochronousIN;
	return true;
}

void CommConnect(ConfiguratorState &state)
{
	if (state.mode != MODE_None) return;

	if (!CommSetup(state))
	{
		wxLogError("ERROR: Failed to setup communication system!");
		return;
	}

	// Detect MarkerTracker device
	if (!comm_check_device(&state.comm))
	{
		wxLogMessage("No Marker Detector connected!");
		return;
	}

	// Connect to detected device
	if (!comm_connect(&state.comm, true))
	{
		wxLogMessage("Failed to connect to Marker Detector!");
		return;
	}

	// Initialize state
	state.mode = MODE_Device;
	state.control.phase = PHASE_None;

	// Start comm thread
	state.runCommThread = true;
	if (state.commThread == NULL) state.commThread = new std::thread(CommThread, &state);

	wxLogMessage("Connected to Marker Detector!");
}

void CommDisconnect(ConfiguratorState &state)
{
	if (state.mode != MODE_Device) return;
	wxLogMessage("Disconnecting!");

	// Join comm thread
	state.runCommThread = false;
	if (state.commThread != NULL && state.commThread->joinable()) state.commThread->join();
	state.commThread = NULL;

	// Reset state
	if (state.control.phase != PHASE_None) StopStreaming(state);
	comm_disconnect(&state.comm);
	state.mode = MODE_None;
}

static void CommThread(ConfiguratorState *state)
{
	while (state->runCommThread)
	{
		// Check connection and connected cameras
		comm_submit_control_request(&state->comm, 2, 0, 0); // Request 2

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}

void StartStreaming(ConfiguratorState &state)
{
	if (state.mode != MODE_Device || state.control.phase != PHASE_None) return;
	wxLogMessage("Starting stream!");

	// Load calibration (camera + markers)
	std::vector<Camera> cameraCalibrations;
	parseCalibrationFile("config/calib.json", cameraCalibrations, state.control.tracking.markerTemplates3D);
	for (int i = 0; i < state.control.tracking.markerTemplates3D.size(); i++)
		generateLookupTables(&state.control.tracking.markerTemplates3D[i]);

	// Set tracking parameters
	state.control.testing.isTesting = false;
	state.control.tracking.sigmaError = state.config.tracking.sigmaError;
	state.control.tracking.minIntersectError = state.config.tracking.minIntersectError;
	state.control.tracking.maxIntersectError = state.config.tracking.maxIntersectError;

	// Setup cameras (TODO: Wait for Marker Tracker to report number of connected cameras)
	state.control.cameras.resize(state.cameraCount);
	for (int i = 0; i < state.cameraCount; i++)
	{
		CameraState *cam = &state.control.cameras[i];
		if (cameraCalibrations.size() > i)
			cam->camera = cameraCalibrations[i];
		else
			wxLogMessage("No calibration found for camera %d!", i);
		cam->camera.width = state.config.mode.cameraResolutionX;
		cam->camera.height = state.config.mode.cameraResolutionY;
		// Setup parameters
		cam->intrinsic.selectionThreshold = state.config.intrinsicCalib.selectionThreshold;
		cam->intrinsic.maxMarkerCount = state.config.intrinsicCalib.maxMarkerCount;
		cam->intrinsic.radialDensityGranularity = state.config.intrinsicCalib.radialDensityGranularity;
		cam->intrinsic.radialDensityTarget = state.config.intrinsicCalib.radialDensityTarget;
		cam->intrinsic.gridSize = state.config.intrinsicCalib.gridSize;
		cam->intrinsic.gridCountTarget = state.config.intrinsicCalib.gridCountTarget;
	}

	// Initialize state
	state.control.phase = PHASE_Idle;

	// Start accepting transfers from Marker Detector
	if (!comm_startStream(&state.comm))
	{
		wxLogMessage("Failed to start streaming!");
		return;
	}

	// Set setup parameters for Marker Detector
	uint16_t setupData[4];
	setupData[0] = state.config.mode.cameraResolutionX;
	setupData[1] = state.config.mode.cameraResolutionY;
	setupData[2] = state.config.mode.cameraFramerate;
	setupData[3] = state.config.mode.cameraShutterSpeed;

	// Tell Marker Detector to start streaming, including setup data
	comm_submit_control_data(&state.comm, 0, 0, 0, setupData, sizeof(setupData)); // request 0

	wxLogMessage("Started stream!");
}

void StopStreaming(ConfiguratorState &state)
{
	if (state.mode != MODE_Device || state.control.phase == PHASE_None) return;
	wxLogMessage("Stopping stream!");
	// Tell Marker Detector to stop streaming
	comm_submit_control_data(&state.comm, 1, 0, 0); // request 1
	// Stop receiving transfers from Marker Detector
	comm_stopStream(&state.comm);
	// Clear resources
	state.control.cameras.clear();
	// Enter phase
	DiscardPhase(&state.control);
	state.control.phase = PHASE_None;
	wxLogMessage("Stopped stream!");
}

void StartTesting(ConfiguratorState &state)
{
	if (state.mode != MODE_None) return;
	wxLogMessage("Starting testing!");

	// Load calibration (camera + markers)
	std::vector<Camera> cameraCalibrations;
	parseCalibrationFile("config/calib.json", cameraCalibrations, state.control.tracking.markerTemplates3D);
	for (int i = 0; i < state.control.tracking.markerTemplates3D.size(); i++)
		generateLookupTables(&state.control.tracking.markerTemplates3D[i]);

	// Set testing & tracking parameters
	state.control.testing.isTesting = true;
	state.control.testing.blobPxStdDev = state.config.testing.blobPxStdDev;
	state.control.tracking.sigmaError = state.config.tracking.sigmaError;
	state.control.tracking.minIntersectError = state.config.tracking.minIntersectError;
	state.control.tracking.maxIntersectError = state.config.tracking.maxIntersectError;

	// Add testing cameras
	state.control.cameras.resize(state.config.testing.cameraDefinitions.size());
	for (int i = 0; i < state.config.testing.cameraDefinitions.size(); i++)
	{
		CameraState *cam = &state.control.cameras[i];
		// Ground truth testing camera
		cam->testing.camera = state.config.testing.cameraDefinitions[i];
		cam->testing.camera.width = state.config.mode.cameraResolutionX;
		cam->testing.camera.height = state.config.mode.cameraResolutionY;
		// Calibrated actual camera
		if (cameraCalibrations.size() > i)
			cam->camera = cameraCalibrations[i];
		//else // Fall back to ground truth
		//	cam->camera = state.config.testing.cameraDefinitions[i];
		cam->camera.width = state.config.mode.cameraResolutionX;
		cam->camera.height = state.config.mode.cameraResolutionY;
		// Setup parameters
		cam->intrinsic.selectionThreshold = state.config.intrinsicCalib.selectionThreshold;
		cam->intrinsic.maxMarkerCount = state.config.intrinsicCalib.maxMarkerCount;
		cam->intrinsic.radialDensityGranularity = state.config.intrinsicCalib.radialDensityGranularity;
		cam->intrinsic.radialDensityTarget = state.config.intrinsicCalib.radialDensityTarget;
		cam->intrinsic.gridSize = state.config.intrinsicCalib.gridSize;
		cam->intrinsic.gridCountTarget = state.config.intrinsicCalib.gridCountTarget;
	}

	// Add testing markers for which there isn't already a calibration
	auto *markers = &state.control.tracking.markerTemplates3D;
	for (int i = 0; i < state.config.testing.trackingMarkers.size(); i++)
	{
		DefMarker *markerDef = &state.config.testing.trackingMarkers[i];

		// Check if this marker has been calibrated yet
		auto existingCalib = std::find_if(markers->begin(), markers->end(), [markerDef](auto &m){ return m.id == markerDef->id; });
		if (existingCalib != markers->end())
		{ // Determine offset of calibrated marker template compared to ground truth
			wxLogMessage("Using calibrated marker template %d for testing marker %s (%d)!", existingCalib->id, markerDef->label,markerDef->id);

			// Set ground truth marker as point cloud
			std::vector<TriangulatedPoint> pointCloud;
			pointCloud.reserve(markerDef->points.size());
			for (int i = 0; i < markerDef->points.size(); i++)
				pointCloud.push_back({ markerDef->points[i].pos, state.control.tracking.maxIntersectError, 1 });

			// Detect match using calibrated marker
			auto pose = detectMarker3D(existingCalib._Ptr, pointCloud, {}, pointCloud.size(), state.control.tracking.sigmaError*2, false);
			if (std::get<2>(pose) > 0)
			{ // Read out offset transform and correct for it to get accurate error calculations
				for (int j = 0; j < existingCalib->points.size(); j++)
					existingCalib->points[j] = std::get<0>(pose) * existingCalib->points[j];
				wxLogMessage("Correcting for %.4fmm calibration offset", std::get<0>(pose).translation().norm()*10);
			}

			// No need to re-generate lookup tables for marker detection
		}
		else
		{
			wxLogMessage("Using ground truth marker template for testing marker %s (%d)!", markerDef->label, markerDef->id);

			// Add ground truth as marker template
			markers->push_back({});
			MarkerTemplate3D *marker = &markers->at(markers->size()-1);
			marker->label = markerDef->label;
			marker->id = markerDef->id;
			marker->points.reserve(markerDef->points.size());
			for (int j = 0; j < markerDef->points.size(); j++)
				marker->points.push_back(markerDef->points[j].pos);

			// Generate lookup tables for marker detection
			generateLookupTables(marker);
		}
	}

	// Initialize state
	state.mode = MODE_Testing;
	state.control.phase = PHASE_Idle;

	// Start test data thread
	state.runTestThread = true;
	if (state.testThread == NULL) state.testThread = new std::thread(TestThread, &state);
}

void StopTesting(ConfiguratorState &state)
{
	if (state.mode != MODE_Testing) return;
	wxLogMessage("Stop testing phase!");

	// Join test thread
	state.runTestThread = false;
	if (state.testThread != NULL && state.testThread->joinable()) state.testThread->join();
	state.testThread = NULL;

	// Reset state
	state.control.cameras.clear();
	state.mode = MODE_None;
	DiscardPhase(&state.control);
}


// ----------------------------------------------------------------------------
// Test Data Thread
// ----------------------------------------------------------------------------

static void TestThread(ConfiguratorState *state)
{
	ControlState *control = &state->control;

	// Render all once at start, mostly to intitialize resources
	if (state->OnUpdateCam)
	{
		for (int m = 0; m < control->cameras.size(); m++)
			state->OnUpdateCam(m);
	}

	control->testing.targetPose = Eigen::Isometry3f::Identity();
	control->testing.GT = Eigen::Isometry3f::Identity();

	int index = 0;
	while (state->runTestThread)
	{
//		if (state->updateTestThread)
		{
			state->updateTestThread = false;
			index = 0;

			// ----- Generate marker pose -----

			auto genPoseInCamView = [](Camera *camera, float min, float max)
			{ // Generate pose facing camera withing range min,max
				const float fov = 1.0f, edge = 1.5f, facing = 40, frontal = 1.6f;
				auto genExp = [](float pow){
					float rnd = rand()%10000 / 5000.0f - 1;
					float exp = std::powf(std::abs(rnd), pow);
					return rnd > 0? exp : -exp;
				};
				Eigen::Vector3f tgtT, tgtR;
				tgtT.z() = (rand()%10000 / 10000.0f) * -max - min;
				tgtT.x() = genExp(1/edge) * fov * tgtT.z()*std::sin(camera->fovH/180*PI/2);
				tgtT.y() = genExp(1/edge) * fov * tgtT.z()*std::sin(camera->fovV/180*PI/2);
				tgtR = Eigen::Vector3f(
					genExp(frontal) * facing,
					genExp(frontal) * facing,
					(rand()%10000 / 5000.0f - 1) * 180);
				return camera->transform * createModelMatrix(tgtT, getRotationZYX(tgtR / 180.0f * PI));
			};
			auto genPoseInCamViews = [&genPoseInCamView](Camera *cameraA, Camera *cameraB, float min, float max)
			{ // Generate pose facing camera withing range min,max
				Eigen::Isometry3f poseA = genPoseInCamView(cameraA, min, max);
				Eigen::Isometry3f poseB = genPoseInCamView(cameraB, min, max);
				// Difference
				Eigen::Vector3f tDiff = poseA.translation() - poseB.translation();
				Eigen::Matrix3f rDiff = poseA.rotation() * poseB.rotation().transpose();
				Eigen::AngleAxisf rDiffAx = Eigen::AngleAxisf(rDiff);
				// Interpolate
				poseB.translation() += tDiff/2;
				rDiffAx.angle() /= 2;
				poseB.linear() = rDiffAx * poseB.linear();
				return poseB;
			};
			if (control->phase == PHASE_Calibration_Intrinsic || control->phase == PHASE_Calibration_Extrinsic)
			{
				if (control->phase == PHASE_Calibration_Intrinsic || (control->phase == PHASE_Calibration_Extrinsic && control->calibration.relations.size() <= state->focusOnCamera))
				{ // Marker in front of cameras
					Camera *camera = &control->cameras[state->focusOnCamera].testing.camera;
					// Generate initial pose
					if (control->testing.targetPose.translation().norm() == 0)
						control->testing.targetPose = control->testing.GT = genPoseInCamView(camera, 20, 40);
					// Generate new target if target is reached
					if ((control->testing.targetPose.translation() - control->testing.GT.translation()).norm() < 1)
						control->testing.targetPose = genPoseInCamView(camera, 20, 40);
				}
				else if (control->phase == PHASE_Calibration_Extrinsic)
				{ // Marker facing two cameras
					Camera *cameraA = &control->cameras[control->calibration.relations[state->focusOnCamera].camA].testing.camera;
					Camera *cameraB = &control->cameras[control->calibration.relations[state->focusOnCamera].camB].testing.camera;
					// Generate initial pose
					if (control->testing.targetPose.translation().norm() == 0)
						control->testing.targetPose = control->testing.GT = genPoseInCamViews(cameraA, cameraB, 100, 200);
					// Generate new target if target is reached
					if ((control->testing.targetPose.translation() - control->testing.GT.translation()).norm() < 1)
						control->testing.targetPose = genPoseInCamViews(cameraA, cameraB, 100, 200);
				}
				// Calculate direction
				Eigen::Vector3f tDiff = control->testing.targetPose.translation() - control->testing.GT.translation();
				Eigen::Matrix3f rDiff = control->testing.targetPose.rotation() * control->testing.GT.rotation().transpose();
				Eigen::AngleAxisf rDiffAx = Eigen::AngleAxisf(rDiff);
				// Lerp GT pose to target
				const float cmPerSec = 50.0f;
				float lerp = std::min(1.0f, cmPerSec/state->config.mode.cameraFramerate / tDiff.norm());
				control->testing.GT.translation() += tDiff * lerp;
				rDiffAx.angle() *= lerp;
				control->testing.GT.linear() = rDiffAx * control->testing.GT.linear();
			}
			else if (control->phase == PHASE_Calibration_Room)
			{ // Marker at origin
				control->testing.GT = Eigen::Isometry3f::Identity();
			}
			else if (control->phase == PHASE_Tracking || control->phase == PHASE_Calibration_Marker)
			{ // Moving marker
				// Generate consistent movement
				state->TD += Eigen::Vector3f(
					(rand()%10000 / 10000.0f) * 0.1 - 0.05,
					(rand()%10000 / 10000.0f) * 0.1 - 0.05,
					(rand()%10000 / 10000.0f) * 0.1 - 0.05);
				state->RD += Eigen::Vector3f(
					(rand()%10000 / 10000.0f) * 2 - 1,
					(rand()%10000 / 10000.0f) * 2 - 1,
					(rand()%10000 / 10000.0f) * 2 - 1);
				// Dampen movement
				state->TD *= 0.9f;
				state->RD *= 0.95f;
				// Apply
				state->TGT += state->TD;
				state->RGT += state->RD/180.0f*PI;
				control->testing.GT = createModelMatrix(state->TGT, getRotationZYX(state->RGT));
			}

			if (control->phase == PHASE_Calibration_Intrinsic || control->phase == PHASE_Calibration_Extrinsic || control->phase == PHASE_Calibration_Room)
			{		
				// ----- Generate calibration marker data -----

				for (int m = 0; m < control->cameras.size(); m++)
				{
					CameraState *camState = &control->cameras[m];

					// Project marker into camera view (simulated test data)
					camState->points2D.clear();
					camState->pointSizes.clear();
					createMarkerProjection(camState->points2D, camState->pointSizes, camState->testing.markerPtsVisible, *control->calibration.markerTemplate2D, camState->testing.camera, control->testing.GT, control->testing.blobPxStdDev);
					
					// Shuffle only if marker is built-in (so a detection algorithm exists)
					if (isMarkerBuiltIn(control->calibration.markerTemplate2D->id))
						std::random_shuffle(camState->points2D.begin(), camState->points2D.end());
				}

				// ----- Single camera pose estimation -----

				HandleCameraState(&state->control);
				
				// ----- Pose Error Calculation -----

				for (int m = 0; m < control->cameras.size(); m++)
				{
					CameraState *camState = &control->cameras[m];

					for (int p = 0; p < camState->extrinsic.poses.size(); p++)
					{ // Found marker pose using CV, check against GT (ground truth)
						std::pair<float,float> poseError = calculatePoseError(control->testing.GT, camState->testing.camera.transform * camState->extrinsic.poses[p]);
						wxLogMessage(L"Cam %d, Pose %d: MSE %.4fmpx, Error (%.4fmm, %.4f\u00B0)", m, p, camState->extrinsic.posesMSE[0]*1000, poseError.first*10, poseError.second);
					}
				}
			}
			else if (control->phase == PHASE_Tracking || control->phase == PHASE_Calibration_Marker)
			{
				// ----- Generate 3D Marker Data -----

				// For testing of triangulation only
				std::vector<int> visibleCount;
				visibleCount.resize(control->testing.markerTemplate3D->points.size());

				for (int m = 0; m < control->cameras.size(); m++)
				{
					CameraState *camState = &control->cameras[m];

					// Project marker into camera view (simulated test data)
					camState->points2D.clear();
					camState->pointSizes.clear();
					createMarkerProjection(camState->points2D, camState->pointSizes, camState->testing.markerPtsVisible, *control->testing.markerTemplate3D, camState->testing.camera, control->testing.GT, control->testing.blobPxStdDev);

					// For testing only, accumulate by how many cameras a point is seen to determine if it is useful for triangulation
					for (int i = 0; i < visibleCount.size(); i++)
						visibleCount[i] += (int)camState->testing.markerPtsVisible[i];
					
					// Shuffle points around
					std::random_shuffle(camState->points2D.begin(), camState->points2D.end());
				}

				// ----- Triangulation and marker detection -----

				HandleCameraState(&state->control);

				if (control->phase == PHASE_Tracking)
				{
					// ----- Pose Error Calculation and failure analysis -----

					// Add 3D rays to shared list for visualization
					control->testing.rays3D.clear();
					for (int m = 0; m < control->cameras.size(); m++)
					{
						CameraState *camState = &control->cameras[m];
						control->testing.rays3D.insert(control->testing.rays3D.end(), camState->tracking.rays3D.begin(), camState->tracking.rays3D.end());
					}

					// Gather points which could have been triangulated
					std::bitset<MAX_MARKER_POINTS> triangulationMask;
					for (int i = 0; i < visibleCount.size(); i++)
						triangulationMask.set(i, visibleCount[i] >= 2);
					control->testing.triangulatedPoints3D.clear();
					transformMarkerPoints(control->testing.triangulatedPoints3D, triangulationMask, *control->testing.markerTemplate3D, control->testing.GT);

					// Handle detected pose
					bool wrongPose = false;
					if (control->tracking.poses3D.size() > 0)
					{ // Determine pose error
						for (int i = 0; i < control->tracking.poses3D.size(); i++)
						{
							std::pair<float,float> poseError = calculatePoseError(control->testing.GT, control->tracking.poses3D[i]);
							std::pair<float,int> poseMSE = control->tracking.posesMSE[i];
							wxLogMessage(L"Pose %d, Points: %d, MSE: %.4fmm, Error: (%.4fmm, %.4f\u00B0)", i, poseMSE.second, poseMSE.first*10, poseError.first*10, poseError.second);
							if (poseError.first > 1 || poseError.second > 2)
							{ // Pose is wrong
								wxLogMessage("--> WRONG POSE: %d reference points", (int)control->testing.triangulatedPoints3D.size());
								wrongPose = true;
							}
						}
					}
					else
					{ // Pose is missing
						wxLogMessage("--> NO POSE: %d reference points!", (int)control->testing.triangulatedPoints3D.size());
						wrongPose = true;
					}

					static int times = 0;
					if (wrongPose && times++ < 10)
					{ // Debug circumstances using ground truth (assuming calibration is perfect) in case pose is wrong (or missing)
						/*MarkerTemplate3D *markerTemplate = std::find_if(control->tracking.markerTemplates3D.begin(), control->tracking.markerTemplates3D.end(), [control](auto &m){ return m.id == control->testing.markerTemplate3D->id; })._Ptr;
						analyzeTrackingAlgorithm(visibleCount, triangulationMask, control->tracking.points3D, *markerTemplate, control->testing.GT);*/
						// Interrupt a few times to allow for debugging
						if (times < 5)
							wxMessageOutput::Get()->Printf (wxT("Failed to detect pose!"));
					}
				}
			}

			// ----- Visualization -----

			if (state->OnUpdateCam)
			{
				for (int m = 0; m < control->cameras.size(); m++)
					state->OnUpdateCam(m);
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds((1000/state->config.mode.cameraFramerate)-1));
	}
}


// ----------------------------------------------------------------------------
// USB Handlers
// ----------------------------------------------------------------------------

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length, void *userData)
{
	ConfiguratorState *state = (ConfiguratorState*)userData;
	if (request == 0x01)
	{ // Debug Response
		if (length != 0)
		{
			std::stringstream hexBuf;
			printBuffer(hexBuf, data, length);
			for (int i = 0; i < length; i++) if (data[i] == 0) data[i] = '*';
			data[length] = 0;
			wxLogMessage("DEBUG (%d): %s  ||  %s", length, (char*)data, hexBuf.str());
		}
	}
	else if (request == 0x02)
	{ // Debug Response
		if (length == 0 || length < data[0]+1)
		{
			wxLogMessage("Received malformed data of length %d!", length);
			return;
		}
		/*{
			std::stringstream hexBuf;
			printBuffer(hexBuf, data, length);
			wxLogMessage("Camera Status (%d): %s", length, hexBuf.str());
		}*/
		state->cameraCount = 0;
		state->cameraMapping.resize(data[0]);
		bool setupChanged = false;
		for (int i = 0; i < data[0]; i++)
		{
			int mapping = -1;
			if((data[i+1] & 0b11) == 0b11)
				mapping = state->cameraCount++;
			setupChanged = setupChanged || state->cameraMapping[i] != mapping;
			state->cameraMapping[i] = mapping;
		}
		if (setupChanged)
		{
			wxLogMessage("Camera setup: %d/%d cameras connected!", state->cameraCount, (int)state->cameraMapping.size());
		}
	}
	else
	{
		std::stringstream hexBuf;
		printBuffer(hexBuf, data, length);
		wxLogMessage("Control Response %d (%d): %s", request, length, hexBuf.str());

	}
}

static void HandleBlobPacket(ConfiguratorState &state, uint8_t *data, int length)
{
	if (length == 128) return; // TODO: Find out why some iso transfers return with 128 and no data
	if (state.mode != MODE_Device || state.control.phase == PHASE_None)
	{ // Not expecting any transfer
		wxLogMessage("Unexpected transfer IN!");
		// Tell Marker Detector to stop streaming
		comm_submit_control_data(&state.comm, 1, 0, 0); // request 1
		// Stop receiving transfers from Marker Detector
		comm_stopStream(&state.comm);
		return;
	}

#ifdef MEASURE_RECEIVE_RATE
	g_receiveCount++;
	auto now = std::chrono::high_resolution_clock::now();
	if (g_receiveCount > 10)
	{
		UpdateStatValue(&receiveRate, std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastReceiveTime).count() / 1000.0);
		if (g_receiveCount % 1000 == 0)
		{
			wxLogMessage("Receive Rate: %.2fms +-%.2fms - Max %.2fms", receiveRate.avg, receiveRate.diff, receiveRate.max);
			receiveRate.max = 0;
		}
	}
	g_lastReceiveTime = now;
#endif

#ifdef DEBUG_TRANSFER_IN
	std::stringstream hexBuf;
	printBuffer(hexBuf, data, length);
	wxLogMessage("IN (%d): %.*s -- %s", length, length, (char*)data, hexBuf.str());
#endif

	// Interpret packet
	int portNr = data[0] - '0';
	int camNr = state.cameraMapping[portNr];
	int blobCount = (length-usbHeaderSize) / 6;
	if (camNr < 0)
	{
		wxLogMessage("Camera Port %d was previously unassigned!", portNr);
		return;
	}
	if (camNr >= state.control.cameras.size())
	{
		wxLogMessage("Camera %d not existing.", camNr);
		return;
	}

	// Update list of blobs
	CameraState *camState = &state.control.cameras[camNr];
	camState->points2D.clear();
	camState->pointSizes.clear();
	camState->points2D.reserve(blobCount);
	camState->pointSizes.reserve(blobCount);
	for (int i = 0; i < blobCount; i++)
	{
		int pos = usbHeaderSize+i*6;
		uint16_t *blobData = (uint16_t*)&data[pos];
		float x = (float)((double)blobData[0] / 65536.0 * camState->camera.width);
		float y = (float)((double)blobData[1] / 65536.0 * camState->camera.height);
		float s = (float)data[pos+4]/2.0f;
		camState->points2D.push_back(Eigen::Vector2f(x,y));
		camState->pointSizes.push_back(s);
	}

	HandleCameraState(&state.control);

	if (state.OnUpdateCam)
		state.OnUpdateCam(camNr);

#ifdef DEBUG_BLOBS_IN
	if (blobCount > 0)
	{
		std::stringstream blobBuf;
		blobBuf.setf(std::ios::fixed, std::ios::floatfield);
		blobBuf.precision(1);
		for (int i = 0; i < blobCount; i++)
		{
			int pos = usbHeaderSize+i*6;
			uint16_t *blobData = (uint16_t*)&data[pos];
			float posX = (double)blobData[0] / 65536.0 * camState->camera.width, posY = (double)blobData[1] / 65536.0 * camState->camera.height, size = (float)data[pos+4]/2, col = data[pos+5];
			blobBuf << "(" << posX << ", " << posY << ", " << size << ") ";
		}
		wxLogMessage("Blobs IN (%d) %.*s %d Blobs: %s", length, usbHeaderSize, data, blobCount, blobBuf.str());
	}
#endif
}

static void onIsochronousIN(uint8_t *data, int length, void *userData)
{
	HandleBlobPacket(*(ConfiguratorState*)userData, data, length);
}

static void onInterruptIN(uint8_t *data, int length, void *userData)
{
	HandleBlobPacket(*(ConfiguratorState*)userData, data, length);
}