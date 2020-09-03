/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "main.h"

#include <chrono>
#include <atomic>
#include <stdlib.h>
#include <algorithm>

#define MEASURE_RECEIVE_RATE
//#define DEBUG_BLOBS_IN
//#define DEBUG_TRANSFER_IN

#ifdef MEASURE_RECEIVE_RATE
static std::chrono::time_point<std::chrono::steady_clock> g_lastReceiveTime;
static StatValue receiveRate;
static std::atomic<long long> g_receiveCount = 0;
#endif

const int testFrameRate = 30;
const int camWidth = 1640;
const int camHeight = 1232;

enum CustomEvents
{
	ID_Connect = wxID_HIGHEST + 1,
	ID_TestCalibration,
	ID_TestTracking,
	ID_Marker, // Always has to be last since higher IDs are used to identify marker
};

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length);
static void onIsochronousIN(uint8_t *data, int length);
static void onInterruptIN(uint8_t *data, int length);

static bool assureGLInit();
static void assureGLClean();

static void ThreadSendTestData();

inline void printBuffer(std::stringstream &ss, uint8_t *buffer, uint8_t size)
{
	ss << "0x";
	ss.setf(std::ios_base::uppercase);
	ss.fill('0');
	ss.width(2);
	ss.setf(std::ios_base::hex);
	for (int i = 0; i < size; i++)
		ss << buffer[i];
}

// ----------------------------------------------------------------------------
// ConfiguratorApp
// ----------------------------------------------------------------------------

wxIMPLEMENT_APP(ConfiguratorApp);

bool ConfiguratorApp::OnInit()
{
	m_state = STATE_Idle;

	// Read config
	parseConfigFile("config/config.json", &m_config);

	if (m_config.testing.calibrationMarkers.size() > 0)
	{ // Set first as default
		setActiveCalibrationMarker(m_config.testing.calibrationMarkers[0]);
	}
	else
	{ // Set some default Marker
		setActiveCalibrationMarker({ "Default 1-Point", { { Eigen::Vector3f( 0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), 360.0f } } });
	}
	if (m_config.testing.trackingMarkers.size() > 0)
	{ // Set first as default
		setActiveTrackingMarker(m_config.testing.trackingMarkers[0]);
	}
	else
	{ // Set some default Marker
		setActiveTrackingMarker({ "Default 1-Point", { { Eigen::Vector3f( 0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), 360.0f } } });
	}

	// Open main frame
	m_frame = new ConfiguratorFrame();
	return true;
}
int ConfiguratorApp::OnExit()
{
	assureGLClean();
	delete m_glContext;
	// Clean up usb device communication
	comm_disconnect(&m_commState);
	comm_exit(&m_commState);
	return 0;
}
wxGLContext *ConfiguratorApp::GetContext(wxGLCanvas *canvas)
{
	if (!m_glContext) m_glContext = new wxGLContext(canvas);
	return m_glContext;
}
CameraState *ConfiguratorApp::GetCameraState(CameraFrame *frame)
{
	for (int i = 0; i < m_markerDetectors.size(); i++)
	{
		if (m_markerDetectors[i].frame == frame)
			return &m_markerDetectors[i].state;
	}
	return NULL;
}
bool ConfiguratorApp::SetupComm()
{
	// Setup usb device communication
	if (!comm_init(&m_commState)) return false;
	m_commState.onControlResponse = onControlResponse;
	m_commState.onInterruptIN = onInterruptIN;
	m_commState.onIsochronousIN = onIsochronousIN;
	return true;
}
void ConfiguratorApp::Connect()
{
	if (!SetupComm())
	{
		wxLogError("ERROR: Failed to setup communication system!");
		return;
	}
	StopTesting();
	if (m_state == STATE_Connected) return;
	// Try to connect to usb Marker Detector
	if (comm_check_device(&m_commState))
	{
		wxLogMessage("Connecting to Marker Detector...");
		if (comm_connect(&m_commState, true))
		{
			wxLogMessage("Connected to Marker Detector!");
			// Connected, now create resources and create window
			MarkerDetector cam = {};
			cam.frame = new CameraFrame("MakersVR MarkerDetector View");
			m_markerDetectors.push_back(cam);
			m_state = STATE_Connected;
		}
		else
		{
			wxLogMessage("Failed to connect to Marker Detector!");
		}
	}
	else
	{
		wxLogMessage("No Marker Detector connected!");
	}
}
void ConfiguratorApp::Disconnect()
{
	if (m_state != STATE_Connected) return;
	if (!m_commState.usbDeviceActive) return;
	wxLogMessage("Disconnecting!");
	// Disconnect device
	comm_disconnect(&m_commState);
	// Destroy windows
	for (int i = 0; i < m_markerDetectors.size(); i++)
	{
		if (m_markerDetectors[i].frame != NULL)
			m_markerDetectors[i].frame->Destroy();
	}
	// Cleanup resources
	m_markerDetectors.clear();
	m_state = STATE_Idle;
}
void ConfiguratorApp::StartTesting(enum ConfiguratorState testingMode)
{
	if (testingMode != STATE_TestingCalibration && testingMode != STATE_TestingTracking) return;
	Disconnect();
	if (m_state == testingMode) return;
	StopTesting();
	wxLogMessage("Starting testing phase!");
	// Add testing cameras
	for (int i = 0; i < m_config.testing.cameraDefinitions.size(); i++)
	{
		MarkerDetector cam = {};
		cam.state.testing.cameraGT = createModelMatrix(m_config.testing.cameraDefinitions[i].pos, m_config.testing.cameraDefinitions[i].rot);
		std::stringstream dbgPos, dbgRot;
		dbgPos << cam.state.testing.cameraGT.translation().transpose();
		dbgRot << getEulerXYZ(cam.state.testing.cameraGT.rotation()).transpose();
		wxLogMessage("Cam %s Pos: %s, Rot: %s", m_config.testing.cameraDefinitions[i].label, dbgPos.str(), dbgRot.str());
		cam.frame = new CameraFrame(m_config.testing.cameraDefinitions[i].label);
		m_markerDetectors.push_back(cam);
	}
	// Start test data thread
	runTestThread = true;
	if (testThread == NULL) testThread = new std::thread(ThreadSendTestData);
	m_state = testingMode;
}
void ConfiguratorApp::StopTesting()
{
	if (m_state != STATE_TestingCalibration && m_state != STATE_TestingTracking) return;
	wxLogMessage("Stop testing phase!");
	// Join thread
	runTestThread = false;
	if (testThread != NULL && testThread->joinable()) testThread->join();
	testThread = NULL;
	// Destroy test windows
	for (int i = 0; i < m_markerDetectors.size(); i++)
	{
		if (m_markerDetectors[i].frame != NULL)
			m_markerDetectors[i].frame->Destroy();
	}
	// Cleanup resources
	m_markerDetectors.clear();
	m_state = STATE_Idle;
}
void ConfiguratorApp::OnCloseCameraFrame(CameraFrame *frame)
{
	// Remove the closed camera frame
	int remaining = 0;
	for (int i = 0; i < m_markerDetectors.size(); i++)
	{
		if (m_markerDetectors[i].frame == frame)
			m_markerDetectors[i].frame = NULL;
		else if (m_markerDetectors[i].frame != NULL)
			remaining++;
	}
	if (remaining == 0)
	{ // Return to idle state
		Disconnect();
		StopTesting();
	}
}

// ----------------------------------------------------------------------------
// ConfiguratorFrame
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(ConfiguratorFrame, wxFrame)
EVT_MENU(ID_Connect, ConfiguratorFrame::OnConnect)
EVT_MENU(ID_TestCalibration, ConfiguratorFrame::OnStartTesting)
EVT_MENU(ID_TestTracking, ConfiguratorFrame::OnStartTesting)
EVT_MENU(wxID_EXIT, ConfiguratorFrame::OnExit)
EVT_MENU(wxID_ABOUT, ConfiguratorFrame::OnAbout)
EVT_CLOSE(ConfiguratorFrame::OnClose)
wxEND_EVENT_TABLE()

ConfiguratorFrame::ConfiguratorFrame()
	: wxFrame(NULL, wxID_ANY, "MakersVR Configurator", wxPoint(20, 20), wxSize(480, 360))
{
	// Main layout
	wxMenuBar *menuBar = new wxMenuBar;
	wxMenu *menu = new wxMenu();
	menuBar->Append(menu, "&Configurator");
	SetMenuBar(menuBar);
	CreateStatusBar();
	wxTextCtrl *logText = new wxTextCtrl(this, -1, "MakersVR Configurator\n", wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_WORDWRAP | wxTE_READONLY);
	wxLog::SetActiveTarget(new wxLogTextCtrl(logText));

	// Setup menu dropdown
	menu->Append(ID_Connect, "&Connect", "Start connecting to a device");
	menu->AppendSeparator();
	menu->Append(ID_TestCalibration, "&Test Calibration", "Start testing the calibration phase.");
	wxMenu *calibMarkerMenu = new wxMenu();
	menu->AppendSubMenu(calibMarkerMenu, "&Calibration Markers", "Select the marker shape to use for calibration testing.");
	menu->Append(ID_TestTracking, "&Test Tracking", "Start testing the tracking of the selected marker.");
	wxMenu *trackMarkerMenu = new wxMenu();
	menu->AppendSubMenu(trackMarkerMenu, "&Tracking Markers", "Select the marker shape to use for tracking testing.");
	menu->AppendSeparator();
	menu->Append(wxID_ABOUT);
	menu->AppendSeparator();
	menu->Append(wxID_EXIT);

	// Fill calibration marker selection
	for (int i = 0; i < wxGetApp().m_config.testing.calibrationMarkers.size(); i++)
	{
		DefMarker *calibMarker = &wxGetApp().m_config.testing.calibrationMarkers[i];
		std::stringstream menuLabel;
		menuLabel << "&" << calibMarker->label;
		calibMarkerMenu->Append(ID_Marker+i, menuLabel.str(), "Select this calibration marker.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+i);
		wxLogMessage("Read Calibration Marker '%s' with %d points!", calibMarker->label, (int)calibMarker->pts.size());
	}

	// Fill tracking marker selection
	int idOffset = wxGetApp().m_config.testing.calibrationMarkers.size();
	for (int i = 0; i < wxGetApp().m_config.testing.trackingMarkers.size(); i++)
	{
		DefMarker *trackMarker = &wxGetApp().m_config.testing.trackingMarkers[i];
		std::stringstream menuLabel;
		menuLabel << "&" << trackMarker->label;
		trackMarkerMenu->Append(ID_Marker+idOffset+i, menuLabel.str(), "Select this tracking marker.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+idOffset+i);
		wxLogMessage("Read Tracking Marker '%s' with %d points!", trackMarker->label, (int)trackMarker->pts.size());
	}

	Show();
}
void ConfiguratorFrame::OnClose(wxCloseEvent& event)
{
	wxGetApp().StopTesting();
	wxGetApp().Disconnect();
	Destroy();
}
void ConfiguratorFrame::OnExit(wxCommandEvent &event)
{
	wxGetApp().StopTesting();
	wxGetApp().Disconnect();
	Destroy();
}
void ConfiguratorFrame::OnAbout(wxCommandEvent &event)
{
	wxMessageBox("MakersVR Configurator is used to configure the MakersVR Tracking System",
				 "About MakersVR Configurator", wxOK | wxICON_INFORMATION);
}
void ConfiguratorFrame::OnConnect(wxCommandEvent &event)
{
	wxGetApp().Connect();
}
void ConfiguratorFrame::OnDisconnect(wxCommandEvent &event)
{
	wxGetApp().Disconnect();
}
void ConfiguratorFrame::OnStartTesting(wxCommandEvent &event)
{
	wxGetApp().StartTesting(event.GetId() == ID_TestTracking? STATE_TestingTracking : STATE_TestingCalibration);
}
void ConfiguratorFrame::OnStopTesting(wxCommandEvent &event)
{
	wxGetApp().StopTesting();
}
void ConfiguratorFrame::OnSelectMarker(wxCommandEvent &event)
{
	int markerID = event.GetId()-ID_Marker;
	int idOffset = wxGetApp().m_config.testing.calibrationMarkers.size();
	if (markerID < idOffset)
	{ // Dealing with calibration marker
		DefMarker *marker = &wxGetApp().m_config.testing.calibrationMarkers.at(markerID);
		wxLogMessage("Select calibration marker '%s' with %d points!", marker->label, (int)marker->pts.size());
		setActiveCalibrationMarker(*marker);
	}
	else
	{ // Dealing with tracking marker
		DefMarker *marker = &wxGetApp().m_config.testing.trackingMarkers.at(markerID-idOffset);
		wxLogMessage("Select tracking marker '%s' with %d points!", marker->label, (int)marker->pts.size());
		setActiveTrackingMarker(*marker);
	}
}

// ----------------------------------------------------------------------------
// GLContext
// ----------------------------------------------------------------------------

static bool assureGLInit()
{
	static bool GLInit = false;
	if (!GLInit)
	{
		// GLEW Init
		GLenum err = glewInit();
		if (GLEW_OK != err)
		{
			wxLogError("GLEW Init Error %d: %s", err, glewGetErrorString(err));
			return false;
		}

		// Init pose inference
		initTesting();
		initCalibration(camWidth, camHeight);
		initTracking();

		GLInit = true;
	}
	return GLInit;
}
static void assureGLClean()
{
	// Clean pose inference
	cleanTesting();
	cleanCalibration();
	cleanTracking();
}

// ----------------------------------------------------------------------------
// CameraFrame
// ----------------------------------------------------------------------------

CameraFrame::CameraFrame(std::string name = "MakersVR Camera View")
	: wxFrame(NULL, wxID_ANY, name)
{
	SetClientSize(camWidth/2, camHeight/2);
	Bind(wxEVT_CLOSE_WINDOW, &CameraFrame::OnClose, this);
	m_canvas = new wxGLCanvas(this, wxID_ANY, NULL, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE);
	m_canvas->Bind(wxEVT_PAINT, &CameraFrame::OnPaint, this);
	m_canvas->Bind(wxEVT_KEY_DOWN, &CameraFrame::OnKeyDown, this);
	Show();
}
void CameraFrame::OnClose(wxCloseEvent& event)
{
	wxGetApp().OnCloseCameraFrame(this);
	Destroy();
}
void CameraFrame::Repaint()
{
	wxClientDC dc(m_canvas);
	Render();
}
void CameraFrame::AssureInit()
{
	wxClientDC dc(m_canvas);
	wxGetApp().GetContext(m_canvas)->SetCurrent(*m_canvas);
	if (!assureGLInit()) return;
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	m_canvas->SwapBuffers();
}
void CameraFrame::Render()
{
	wxGetApp().GetContext(m_canvas)->SetCurrent(*m_canvas);
	if (!assureGLInit()) return;

	const wxSize ClientSize = GetClientSize() * GetContentScaleFactor();
	glViewport(0, 0, ClientSize.x, ClientSize.y);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	CameraState *camState = wxGetApp().GetCameraState(this);
	if (wxGetApp().m_state == STATE_TestingCalibration)
		visualizePoses(camState->camera, camState->points2D, camState->m_markers, camState->poses);
	else if (wxGetApp().m_state == STATE_TestingTracking)
		visualizeMarkers(camState->camera, camState->points2D, wxGetApp().rays3D, wxGetApp().points3D, wxGetApp().nonconflictedCount, wxGetApp().testing.triangulatedPoints3D, wxGetApp().poses3D);

	m_canvas->SwapBuffers();
}
void CameraFrame::OnPaint(wxPaintEvent &event)
{
//	wxPaintDC dc(m_canvas);
//	Render();
}

static Eigen::Vector3f TGT (0, 0, 100);
static Eigen::Vector3f RGT (0, 0, 0);
static Eigen::Vector3f TD (0, 0, 0);
static Eigen::Vector3f RD (0, 0, 0);
const float dA = 5, dX = 10;
static bool updateTestThread = true;
void CameraFrame::OnKeyDown(wxKeyEvent &event)
{
	CameraState *camState = wxGetApp().GetCameraState(this);
	switch (event.GetKeyCode())
	{
	// Target Marker Rotation
	case 'q': case 'Q':
		RGT.z() += dA/180.0f*PI;
		break;
	case 'e': case 'E':
		RGT.z() -= dA/180.0f*PI;
		break;
	case 'w': case 'W':
		RGT.x() -= dA/180.0f*PI;
		break;
	case 's': case 'S':
		RGT.x() += dA/180.0f*PI;
		break;
	case 'a': case 'A':
		RGT.y() -= dA/180.0f*PI;
		break;
	case 'd': case 'D':
		RGT.y() += dA/180.0f*PI;
		break;
	// Target Marker Position
	case WXK_UP:
		TGT.y() += dX;
		break;
	case WXK_DOWN:
		TGT.y() -= dX;
		break;
	case WXK_LEFT:
		TGT.x() -= dX;
		break;
	case WXK_RIGHT:
		TGT.x() += dX;
		break;
	case WXK_PAGEUP:
		TGT.z() += dX;
		break;
	case WXK_PAGEDOWN:
		TGT.z() -= dX;
		break;
	// Camera Position
	case 'i': case 'I':
		camState->camera.transform.translation().z() -= dX;
		break;
	case 'k': case 'K':
		camState->camera.transform.translation().z() += dX;
		break;
	case 'j': case 'J':
		camState->camera.transform.translation().x() -= dX;
		break;
	case 'l': case 'L':
		camState->camera.transform.translation().x() += dX;
		break;
	// Camera Rotation
	case 't': case 'T':
		camState->camera.transform.linear() = getRotationXYZ(getEulerXYZ(camState->camera.transform.rotation()) + Eigen::Vector3f(+dA/180.0f*PI, 0, 0));
		break;
	case 'g': case 'G':
		camState->camera.transform.linear() = getRotationXYZ(getEulerXYZ(camState->camera.transform.rotation()) + Eigen::Vector3f(-dA/180.0f*PI, 0, 0));
		break;
	case 'h': case 'H':
		camState->camera.transform.linear() = getRotationXYZ(getEulerXYZ(camState->camera.transform.rotation()) + Eigen::Vector3f(0, +dA/180.0f*PI, 0));
		break;
	case 'f': case 'F':
		camState->camera.transform.linear() = getRotationXYZ(getEulerXYZ(camState->camera.transform.rotation()) + Eigen::Vector3f(0, -dA/180.0f*PI, 0));
		break;
	case 'r': case 'R':
		camState->camera.transform.linear() = getRotationXYZ(getEulerXYZ(camState->camera.transform.rotation()) + Eigen::Vector3f(0, 0, +dA/180.0f*PI));
		break;
	case 'z': case 'Z':
		camState->camera.transform.linear() = getRotationXYZ(getEulerXYZ(camState->camera.transform.rotation()) + Eigen::Vector3f(0, 0, -dA/180.0f*PI));
		break;
	default:
		event.Skip();
		return;
	}
	updateTestThread = true;
}

// ----------------------------------------------------------------------------
// Test Data Thread
// ----------------------------------------------------------------------------

static void ThreadSendTestData()
{
	ConfiguratorApp *host = &wxGetApp();

	// Render all once at start, mostly to intitialize resources
	for (int m = 0; m < host->m_markerDetectors.size(); m++)
	{
		MarkerDetector *markerDetector = &host->m_markerDetectors[m];
		if (markerDetector->frame != NULL) markerDetector->frame->AssureInit();
	}

	int index = 0;
	while (host->runTestThread)
	{
//		if (++index >= (1000/testFrameRate)-1)
//		if (updateTestThread)
		{
			updateTestThread = false;
			index = 0;

			// Create random Ground Truth transform

			// Generate completely random poses
			/*TGT.z() = (rand()%10000 / 10000.0f) * 200 + 20;
			TGT.x() = (rand()%10000 / 10000.0f) * TGT.z()/2 - TGT.z()/4;
			TGT.x() = (rand()%10000 / 10000.0f) * TGT.z()/2 - TGT.z()/4;
			RGT = Eigen::Vector3f(
				(rand()%10000 / 10000.0f) * 100 - 50,
				(rand()%10000 / 10000.0f) * 100 - 50,
				(rand()%10000 / 10000.0f) * 100 - 50);
			*/

			// Generate consistently moving pose
			TD += Eigen::Vector3f(
				(rand()%10000 / 10000.0f) * 0.1 - 0.05,
				(rand()%10000 / 10000.0f) * 0.1 - 0.05,
				(rand()%10000 / 10000.0f) * 0.1 - 0.05);
			RD += Eigen::Vector3f(
				(rand()%10000 / 10000.0f) * 2 - 1,
				(rand()%10000 / 10000.0f) * 2 - 1,
				(rand()%10000 / 10000.0f) * 2 - 1);
			// Dampen movement
			TD *= 0.9f;
			RD *= 0.95f;
			// Apply movement
			TGT += TD;
			RGT += RD/180.0f*PI;

			// Transform to Eigen
			Eigen::Matrix3f RGT_Mat = getRotationZYX(RGT);
			Eigen::Isometry3f GT = createModelMatrix(TGT, RGT_Mat);

			if (host->m_state == STATE_TestingCalibration)
			{
				
				// ----- Generate calibration marker data -----

				for (int m = 0; m < host->m_markerDetectors.size(); m++)
				{
					CameraState *camState = &host->m_markerDetectors[m].state;

					// Assume perfect knowledge of camera position (TODO: infer using calibration)
					camState->camera = getCalibratedCamera();
					camState->camera.transform = camState->testing.cameraGT;

					// Project marker into camera view (simulated test data)
					camState->points2D.clear();
					createMarkerProjection(camState->points2D, camState->testing.markerPtsVisible, calibMarker3D, camState->camera, GT, CAMERA_NOISE_STDDEV);
					
					// Assume perfect order of image points
					//std::random_shuffle(camState->points2D.begin(), camState->points2D.end());
				}

				// ----- Single camera pose estimation -----

				for (int m = 0; m < host->m_markerDetectors.size(); m++)
				{
					CameraState *camState = &host->m_markerDetectors[m].state;

					// Single camera pose estimation using OpenCV
					camState->poses.clear();

					// Assume all points visible belong to singular marker (TODO: Marker detection and point ordering)
					//findMarkerCandidates(camState->points2D, camState->m_markers, camState->m_freeBlobs);

					// Infer pose of generic marker under above assumptions
					if (camState->points2D.size() >= getExpectedBlobCount())
					{
						Eigen::Isometry3f pose;
						inferMarkerPoseGeneric(camState->points2D, camState->camera, pose);
						camState->poses.push_back(pose);
					}

					// Infer pose of detected markers
					//inferMarkerPoses(camState->m_markers, camState->camera, camState->poses);
				}

				// ----- Pose Error Calculation -----

				for (int m = 0; m < host->m_markerDetectors.size(); m++)
				{
					CameraState *camState = &host->m_markerDetectors[m].state;

					if (camState->poses.size() > 0)
					{ // Found marker pose using CV, check against GT (ground truth)
						Eigen::Vector3f tDiff = camState->poses[0].translation()-TGT;
						Eigen::Matrix3f rDiff = RGT_Mat * camState->poses[0].rotation().transpose();
						float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
						wxLogMessage(L"Cam %d: CV Translation Error: %fcm -- Angle Error: %f\u00B0", m, tError, rError);
					}

					// Add origin for orientation
					camState->poses.push_back(Eigen::Isometry3f::Identity());
				}
			}
			else if (host->m_state == STATE_TestingTracking)
			{

				// ----- Generate 3D Marker Data -----

				// For testing of triangulation only
				std::vector<int> visibleCount;
				visibleCount.resize(marker3D.markerTemplate.pts.size());
				
				for (int m = 0; m < host->m_markerDetectors.size(); m++)
				{
					CameraState *camState = &host->m_markerDetectors[m].state;

					// Assume perfect knowledge of camera position (TODO: infer using calibration)
					camState->camera = getCalibratedCamera();
					camState->camera.transform = camState->testing.cameraGT;

					// Project marker into camera view (simulated test data)
					camState->points2D.clear();
					createMarkerProjection(camState->points2D, camState->testing.markerPtsVisible, marker3D.markerTemplate, camState->camera, GT, CAMERA_NOISE_STDDEV);

					// For testing only, accumulate by how many cameras a point is seen to determine if it is useful for triangulation
					for (int i = 0; i < visibleCount.size(); i++)
						visibleCount[i] += (int)camState->testing.markerPtsVisible[i];
					
					// Shuffle points around
					std::random_shuffle(camState->points2D.begin(), camState->points2D.end());
				}

				// ----- Triangulation and marker detection -----

				for (int m = 0; m < host->m_markerDetectors.size(); m++)
				{
					CameraState *camState = &host->m_markerDetectors[m].state;

					// Create 3D Rays for triangulation assuming calibrated camera
					camState->rays3D.clear();
					castRays(camState->points2D, camState->camera, camState->rays3D);
				}

				// Perform triangulation using 3D Rays
				std::vector<std::vector<Ray>*> rayGroups;
				for (int m = 0; m < host->m_markerDetectors.size(); m++)
					rayGroups.push_back(&host->m_markerDetectors[m].state.rays3D);
				host->points3D.clear();
				host->conflicts.clear();
				host->nonconflictedCount = triangulateRayIntersections(rayGroups, host->points3D, host->conflicts, 0.01f);

				// Detect markers in triangulated point cloud
				host->poses3D.clear();
				detectMarkers3D(host->points3D, host->conflicts, host->nonconflictedCount, host->poses3D);

				// ----- Pose Error Calculation and failure analysis -----

				// Add 3D rays to shared list for visualization
				host->rays3D.clear();
				for (int m = 0; m < host->m_markerDetectors.size(); m++)
				{
					CameraState *camState = &host->m_markerDetectors[m].state;
					host->rays3D.insert(host->rays3D.end(), camState->rays3D.begin(), camState->rays3D.end());
				}

				// Gather points which can be triangulated
				std::bitset<MAX_MARKER_POINTS> triangulationMask;
				for (int i = 0; i < visibleCount.size(); i++)
					triangulationMask.set(i, visibleCount[i] >= 2);
				transformMarkerPoints(host->testing.triangulatedPoints3D, triangulationMask, GT);

				// Handle detected pose
				bool wrongPose = false;
				if (host->poses3D.size() > 0)
				{ // Determine pose error
					Eigen::Isometry3f pose = host->poses3D[0];
					Eigen::Vector3f tDiff = pose.translation() - TGT;
					Eigen::Matrix3f rDiff = RGT_Mat * pose.rotation().transpose();
					float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
					wxLogMessage(L"--> Pose Error: Pos %fcm -- Angle %f\u00B0", tError, rError);
					if (tError > 1 || rError > 2)
					{ // Pose is wrong
						wxLogMessage("Pose wrongly detected! Had %d true triangulated marker points!", (int)host->testing.triangulatedPoints3D.size());
						wrongPose = true;
					}
				}
				else
				{ // Pose is missing
					wxLogMessage("Pose not detected! Had %d true triangulated marker points!", (int)host->testing.triangulatedPoints3D.size());
					wrongPose = true;
				}

				if (wrongPose)
				{ // Debug circumstances using ground truth in case pose is wrong (or missing)
					analyzeTrackingAlgorithm(visibleCount, triangulationMask, host->points3D, GT);
					// Interrupt a few times to allow for debugging
					static int times = 0;
					if (times++ < 5) wxMessageOutput::Get()->Printf (wxT("Failed to detect pose!"));
				}

				// Add origin for orientation
				host->poses3D.push_back(Eigen::Isometry3f::Identity());
			}

			// ----- Visualization -----

			for (int m = 0; m < host->m_markerDetectors.size(); m++)
			{
				MarkerDetector *markerDetector = &host->m_markerDetectors[m];
				if (markerDetector->frame != NULL) markerDetector->frame->Repaint();
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds((1000/testFrameRate)-1));
	}
}

// ----------------------------------------------------------------------------
// USB Handlers
// ----------------------------------------------------------------------------

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length)
{
	if (request == 0x01)
	{ // Debug Response
		if (length != 0)
		{
			//std::stringstream hexBuf;
			//printBuffer(hexBuf, data, length);
			//wxLogMessage("DEBUG (%d): %s", length, hexBuf.str());
			for (int i = 0; i < length; i++) if (data[i] == 0) data[i] = '*';
			data[length] = 0;
			wxLogMessage("DEBUG (%d): %s", length, data);
		}
	}
	else
	{
		std::stringstream hexBuf;
		printBuffer(hexBuf, data, length);
		wxLogMessage("Control Response %d (%d): %s", request, length, hexBuf.str());

	}
}

static void onIsochronousIN(uint8_t *data, int length)
{

#ifdef MEASURE_RECEIVE_RATE
	g_receiveCount++;
	auto now = std::chrono::high_resolution_clock::now();
	//if (std::chrono::duration_cast<std::chrono::hours>(std::chrono::time_point<std::chrono::high_resolution_clock>::max() - g_lastReceiveTime).count() > 1)
	if (g_receiveCount > 10)
	{
		UpdateStatValue(&receiveRate, std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastReceiveTime).count() / 1000.0);
		if (g_receiveCount % 10 == 0)
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
	wxLogMessage("Isochronous IN (%d): %.*s -- %s", length, length, (char*)data, hexBuf.str());
#endif

	const int headerSize = 8;
	static int blobUpdate = 0;
	blobUpdate++;
	int blobCount = (length-headerSize) / 6;

	MarkerDetector *markerDetector = &wxGetApp().m_markerDetectors[0];
	CameraState *camState = &markerDetector->state;

	// Update list of blobs
	camState->points2D.resize(blobCount);
	for (int i = 0; i < blobCount; i++)
	{
		Point *point = &camState->points2D[i];
		int pos = headerSize+i*6;
		uint16_t *blobData = (uint16_t*)&data[pos];
		point->X = (float)((double)blobData[0] / 65536.0 * camWidth);
		point->Y = (float)((double)blobData[1] / 65536.0 * camHeight);
		point->S = (float)data[pos+4]/2.0f;
		//col = data[pos+5];
	}

	// Get list of (potential) markers
	findMarkerCandidates(camState->points2D, camState->m_markers, camState->m_freeBlobs);

	// Infer the pose of these markers
	inferMarkerPoses(camState->m_markers, camState->camera, camState->poses);

	if (markerDetector->frame != NULL)
		markerDetector->frame->Repaint();

#ifdef DEBUG_BLOBS_IN
	if (blobCount > 0 || blobUpdate % 100 == 0)
	{
		std::stringstream blobBuf;
		blobBuf.setf(std::ios_base::fixed, std::ios_base::floatfield);
		blobBuf.precision(1);
		for (int i = 0; i < blobCount; i++)
		{
			int pos = headerSize+i*6;
			uint16_t *blobData = (uint16_t*)&data[pos];
			float posX = (double)blobData[0] / 65536.0, posY = (double)blobData[1] / 65536.0, size = (float)data[pos+4]/2, col = data[pos+5];
			blobBuf << "(" << posX * 2048 << ", " << posY * 2048 << ", " << size << ") ";
		}
		wxLogMessage("Isochronous IN (%d) %.*s %d Blobs: %s", length, headerSize, data, blobCount, blobBuf.str());
	}
#endif
}

static void onInterruptIN(uint8_t *data, int length)
{
#ifdef MEASURE_RECEIVE_RATE
	g_receiveCount++;
	auto now = std::chrono::high_resolution_clock::now();
	//if (std::chrono::duration_cast<std::chrono::hours>(std::chrono::time_point<std::chrono::high_resolution_clock>::max() - g_lastReceiveTime).count() > 1)
	if (g_receiveCount > 10)
	{
		UpdateStatValue(&receiveRate, std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastReceiveTime).count() / 1000.0);
		if (g_receiveCount % 10 == 0)
		{
			wxLogMessage("Receive Rate: %.2fms (%.2fms +-%.2fms) - Max %.2fms", receiveRate.cur, receiveRate.avg, receiveRate.diff, receiveRate.max);
			receiveRate.max = 0;
		}
	}
	g_lastReceiveTime = now;
#endif

#ifdef DEBUG_TRANSFER_IN
	std::stringstream hexBuf;
	printBuffer(hexBuf, data, length);
	wxLogMessage("Interrupt IN (%d): %.*s -- %s", length, length, (char*)data, hexBuf.str());
#endif

#ifdef DEBUG_BLOBS_IN
	if (length == 3 && data[0] == 'N' && data[1] == 'C' && data[2] == 'D')
	{ // No connected detectors
		wxLogMessage("Marker Tracker reports no connected Marker Detectors!");
	}
	else if (length == 3 && data[0] == 'S' && data[1] == 'T' && data[2] == 'L')
	{ // No connected detectors
		wxLogMessage("Marker Tracker reports stalled Marker Detectors!");
	}
	else
	{
		static int blobUpdate = 0;
		blobUpdate++;
		int blobCount = (length-4) / 6;
		char message[5] = {0};
		memcpy(message, data, 4);
		if (blobCount > 0 || blobUpdate % 10 == 0)
		{
			std::stringstream hexBuf;
			printBuffer(hexBuf, data+4, length-4);
			wxLogMessage("Interrupt IN (%d) %s %d Blobs: %s", length, message, blobCount, hexBuf.str());
		}
	}
#endif
}