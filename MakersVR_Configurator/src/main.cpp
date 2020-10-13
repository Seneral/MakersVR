/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "GL/glew.h"

#include "main.hpp"

#include "testing.hpp"

#include <chrono>
#include <bitset>

#define MEASURE_RECEIVE_RATE
//#define DEBUG_BLOBS_IN
//#define DEBUG_TRANSFER_IN

#ifdef MEASURE_RECEIVE_RATE
static std::chrono::time_point<std::chrono::steady_clock> g_lastReceiveTime;
static StatValue receiveRate;
static std::atomic<long long> g_receiveCount = 0;
#endif

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

	// Get built-in calibration markers
	getBuiltInMarkers(m_globalState.calibration.markerTemplates2D);

	// Copy testing markers into database
	int prevCalibCnt = m_globalState.calibration.markerTemplates2D.size();
	m_globalState.calibration.markerTemplates2D.reserve(prevCalibCnt + m_config.testing.calibrationMarkers.size());
	for (int i = 0; i < m_config.testing.calibrationMarkers.size(); i++)
	{
		m_globalState.calibration.markerTemplates2D.push_back(m_config.testing.calibrationMarkers[i]);
		m_globalState.calibration.markerTemplates2D[prevCalibCnt+i].id = prevCalibCnt+i;
	}
	if (m_globalState.calibration.markerTemplates2D.size() > 0) m_globalState.calibration.markerTemplate2D = &m_globalState.calibration.markerTemplates2D[0];

	// Copy testing markers into database
	int prevTrackCnt = m_globalState.tracking.markerTemplates3D.size();
	m_globalState.tracking.markerTemplates3D.resize(prevTrackCnt + m_config.testing.trackingMarkers.size());
	for (int i = 0; i < m_config.testing.trackingMarkers.size(); i++)
	{
		DefMarker *markerDef = &m_config.testing.trackingMarkers[i];
		MarkerTemplate3D *marker = &m_globalState.tracking.markerTemplates3D[prevTrackCnt+i];
		marker->label = markerDef->label;
		marker->id = -i-1;
		marker->points.reserve(markerDef->points.size());
		for (int j = 0; j < markerDef->points.size(); j++)
			marker->points.push_back(markerDef->points[j].pos);
		generateLookupTables(marker);
	}
	if (m_globalState.tracking.markerTemplates3D.size() > 0) m_globalState.tracking.markerTemplate3D = &m_globalState.tracking.markerTemplates3D[0];

	// TODO: Copy calibrated tracking markers into database

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
	for (int i = 0; i < m_globalState.cameras.size(); i++)
	{
		if (m_cameraFrames[i] == frame)
			return &m_globalState.cameras[i];
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
			m_globalState = {};
			// Load calibration
			std::vector<Camera> cameraCalibrations;
			parseCalibrationFile("config/calib.json", cameraCalibrations);
			m_globalState.cameras.resize(cameraCalibrations.size());
			for (int i = 0; i < cameraCalibrations.size(); i++)
			{
				CameraState *cam = &m_globalState.cameras[i];
				cam->camera = cameraCalibrations[i];
				cam->camera.width = m_config.mode.cameraResolutionX;
				cam->camera.height = m_config.mode.cameraResolutionY;
				m_cameraFrames.push_back(new CameraFrame("MakersVR MarkerDetector View"));
			}
			// Enter phase
			m_state = STATE_Connected;
			EnterPhase(&m_globalState, PHASE_None);
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
	for (int i = 0; i < m_cameraFrames.size(); i++)
	{
		if (m_cameraFrames[i] != NULL)
			m_cameraFrames[i]->Destroy();
	}
	// Cleanup resources
	m_cameraFrames.clear();
	m_globalState.cameras.clear();
	m_state = STATE_Idle;
	EnterPhase(&m_globalState, PHASE_None);
}
void ConfiguratorApp::StartTesting(enum ControlPhase phase)
{
	if (m_state == STATE_Testing && m_globalState.phase == phase) return;
	Disconnect();
	StopTesting();
	wxLogMessage("Starting testing phase %d!", (int)phase);
	// Load calibration
	std::vector<Camera> cameraCalibrations;
	parseCalibrationFile("config/calib.json", cameraCalibrations);
	if (cameraCalibrations.size() < m_config.testing.cameraDefinitions.size())
		phase = PHASE_Calibration_Intrinsic; // Force calibration
	// Set testing&tracking parameters
	m_globalState.testing.blobPxStdDev = m_config.testing.blobPxStdDev;
	m_globalState.tracking.sigmaError = m_config.tracking.sigmaError;
	m_globalState.tracking.intersectError = m_config.tracking.intersectError;
	// Add testing cameras
	m_globalState.cameras.resize(m_config.testing.cameraDefinitions.size());
	for (int i = 0; i < m_config.testing.cameraDefinitions.size(); i++)
	{
		DefCamera *def = &m_config.testing.cameraDefinitions[i];
		CameraState *cam = &m_globalState.cameras[i];
		// Ground truth testing camera
		cam->testing.camera.width = m_config.mode.cameraResolutionX;
		cam->testing.camera.height = m_config.mode.cameraResolutionY;
		cam->testing.camera.fovH = def->fovH;
		cam->testing.camera.fovV = def->fovV;
		memcpy(&cam->testing.camera.distortion, def->distortion, sizeof(cam->testing.camera.distortion));
		//cam.testing.camera.distortion = { 0.132183f, -0.186451f, 0.001267f, 0.000618f, 0.000000f };
		cam->testing.camera.transform = createModelMatrix(def->pos, def->rot);
		// Calibrated actual camera
		if (cameraCalibrations.size() > i)
			cam->camera = cameraCalibrations[i];
		cam->camera.width = m_config.mode.cameraResolutionX;
		cam->camera.height = m_config.mode.cameraResolutionY;
		// Register and create frame
		m_cameraFrames.push_back(new CameraFrame(def->label));

		{
			// Debug
			Camera *tCam = &cam->testing.camera;
			Camera *cCam = &cam->camera;
			wxLogMessage("Cam %d labeled %s", i, m_config.testing.cameraDefinitions[i].label);
			wxLogMessage("Cam %d testing intrinsics: FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", i, tCam->fovH, tCam->fovV,
				tCam->distortion.k1, tCam->distortion.k2, tCam->distortion.p1, tCam->distortion.p2, tCam->distortion.k3);
			if (cCam->fovH != 0)
			{
				wxLogMessage("Cam %d calibrated intrinsics: FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", i, cCam->fovH, cCam->fovV,
					cCam->distortion.k1, cCam->distortion.k2, cCam->distortion.p1, cCam->distortion.p2, cCam->distortion.k3);
			}
			Eigen::Vector3f posGT = tCam->transform.translation();
			Eigen::Vector3f rotGT = getEulerXYZ(tCam->transform.rotation());
			wxLogMessage("Cam %d testing transform: Pos (%.2f, %.2f, %.2f), Rot (%.2f, %.2f, %.2f)", i,
				posGT.x(), posGT.y(), posGT.z(), rotGT.x(), rotGT.y(), rotGT.z());
			if (cCam->transform.translation().sum() != 0)
			{
				Eigen::Vector3f posCB = cCam->transform.translation();
				Eigen::Vector3f rotCB = getEulerXYZ(cCam->transform.rotation());
				Eigen::Vector3f tDiff = cCam->transform.translation() - tCam->transform.translation();
				Eigen::Matrix3f rDiff = tCam->transform.rotation() * cCam->transform.rotation().transpose();
				float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
				wxLogMessage("Cam %d calibrated transform: Pos (%.2f, %.2f, %.2f), Rot (%.2f, %.2f, %.2f), Error: (%.4fmm, %.4f\u00B0)", i,
					posCB.x(), posCB.y(), posCB.z(), rotCB.x(), rotCB.y(), rotCB.z(), tError*10, rError);
			}
		}
	}
	// Initialize state
	m_state = STATE_Testing;
	EnterPhase(&m_globalState, phase);
	// Start test data thread
	runTestThread = true;
	if (testThread == NULL) testThread = new std::thread(ThreadSendTestData);
}
void ConfiguratorApp::StopTesting()
{
	if (m_state != STATE_Testing) return;
	wxLogMessage("Stop testing phase %d!", m_globalState.phase);
	// Join thread
	runTestThread = false;
	if (testThread != NULL && testThread->joinable()) testThread->join();
	testThread = NULL;
	// Destroy windows
	for (int i = 0; i < m_cameraFrames.size(); i++)
	{
		if (m_cameraFrames[i] != NULL)
			m_cameraFrames[i]->Destroy();
	}
	// Cleanup resources
	m_cameraFrames.clear();
	m_globalState.cameras.clear();
	m_state = STATE_Idle;
	EnterPhase(&m_globalState, PHASE_None);
}
void ConfiguratorApp::OnCloseCameraFrame(CameraFrame *frame)
{
	// Remove the closed camera frame
	int remaining = 0;
	for (int i = 0; i < m_cameraFrames.size(); i++)
	{
		if (m_cameraFrames[i] == frame)
			m_cameraFrames[i] = NULL;
		else if (m_cameraFrames[i] != NULL)
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

	TrackingState *state = &wxGetApp().m_globalState;

	// Fill calibration marker selection (built-in + config)
	for (int i = 0; i < state->calibration.markerTemplates2D.size(); i++)
	{
		DefMarker *calibMarker = &state->calibration.markerTemplates2D[i];
		calibMarkerMenu->Append(ID_Marker+i, calibMarker->label, "Select this calibration marker.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+i);
		wxLogMessage("Read Calibration Marker '%s' (%d) with %d points!", calibMarker->label, calibMarker->id, (int)calibMarker->points.size());
	}

	// Fill tracking marker selection (only config, not calibrated markers)
	int idOffset = state->calibration.markerTemplates2D.size();
	for (int i = 0; i < state->tracking.markerTemplates3D.size(); i++)
	{
		MarkerTemplate3D *trackMarker = &state->tracking.markerTemplates3D[i];
		if (trackMarker->id >= 0) continue; // Can't select calibrated markers, since there is not enough information (LED direction, fov) to simulate it
		trackMarkerMenu->Append(ID_Marker+idOffset+i, trackMarker->label, "Select this tracking marker.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+idOffset+i);
		wxLogMessage("Read Tracking Marker '%s' (%d) with %d points!", trackMarker->label, trackMarker->id, (int)trackMarker->points.size());
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
	wxGetApp().StartTesting(event.GetId() == ID_TestTracking? PHASE_Tracking : PHASE_Calibration_Intrinsic);
}
void ConfiguratorFrame::OnStopTesting(wxCommandEvent &event)
{
	wxGetApp().StopTesting();
}
void ConfiguratorFrame::OnSelectMarker(wxCommandEvent &event)
{
	TrackingState *state = &wxGetApp().m_globalState;
	int markerID = event.GetId()-ID_Marker;
	int idOffset = state->calibration.markerTemplates2D.size();
	if (markerID < idOffset)
	{ // Dealing with calibration marker
		state->calibration.markerTemplate2D = &state->calibration.markerTemplates2D[markerID];
		wxLogMessage("Select calibration marker %s (%d) with %d points!", state->calibration.markerTemplate2D->label, state->calibration.markerTemplate2D->id, (int)state->calibration.markerTemplate2D->points.size());
	}
	else
	{ // Dealing with tracking marker
		state->tracking.markerTemplate3D = &state->tracking.markerTemplates3D[markerID-idOffset];
		wxLogMessage("Select tracking marker %s (%d) with %d points!", state->tracking.markerTemplate3D->label, state->tracking.markerTemplate3D->id, (int)state->tracking.markerTemplate3D->points.size());
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
		initVisualization();

		GLInit = true;
	}
	return GLInit;
}
static void assureGLClean()
{
	cleanVisualization();
}

// ----------------------------------------------------------------------------
// CameraFrame
// ----------------------------------------------------------------------------

CameraFrame::CameraFrame(std::string name = "MakersVR Camera View")
	: wxFrame(NULL, wxID_ANY, name)
{
	SetClientSize(800, 600);
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

	TrackingState *state = &wxGetApp().m_globalState;
	CameraState *cam = wxGetApp().GetCameraState(this);
	if (state->phase == PHASE_Calibration_Intrinsic)
	{
		visualizeDistortion(cam->camera, cam->testing.camera);
		visualizePoints2D(cam->camera, cam->points2D);
		visualizeMarkers(cam->camera, cam->intrinsic.markers);
	}
	else if (state->phase == PHASE_Calibration_Extrinsic || state->phase == PHASE_Calibration_Room)
	{
//		visualizeDistortion(cam->camera, cam->testing.camera);
		visualizePoints2D(cam->camera, cam->undistorted2D);
		visualizePoses(cam->camera, cam->extrinsic.poses, true);
		visualizeMarkers(cam->camera, cam->extrinsic.markers);
	}
	else if (state->phase == PHASE_Calibration_Marker)
	{
		// TODO
	}
	else if (state->phase == PHASE_Tracking)
	{
		visualizePoints2D(cam->camera, cam->undistorted2D, { 1, 1, 1 });
		visualizeRays(cam->camera, state->testing.rays3D);
		visualizePoses(cam->camera, state->tracking.poses3D, false);
		visualizeTriangulation(cam->camera, state->tracking.points3D, state->tracking.nonconflictedCount);
//		visualizePoints3D(cam->camera, state->testing.triangulatedPoints3D, {0,1,0}, 2.0f);
	}
	else if (state->phase == PHASE_None)
		visualizePoints2D(cam->camera, cam->points2D);

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
static int focusOnCamera = 0;
void CameraFrame::OnKeyDown(wxKeyEvent &event)
{
	TrackingState *state = &wxGetApp().m_globalState;
	CameraState *cam = wxGetApp().GetCameraState(this);
	switch (event.GetKeyCode())
	{
	// State control
	case 'b': case 'B': // Finalize phase
		FinalizePhase(state);
		break;
	case 'n': case 'N': // Next phase
		NextPhase(state);
		break;
	case '1':
		focusOnCamera = 0;
		break;
	case '2':
		focusOnCamera = 1;
		break;
	case '3':
		focusOnCamera = 2;
		break;
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
		cam->camera.transform.translation().z() -= dX;
		break;
	case 'k': case 'K':
		cam->camera.transform.translation().z() += dX;
		break;
	case 'j': case 'J':
		cam->camera.transform.translation().x() -= dX;
		break;
	case 'l': case 'L':
		cam->camera.transform.translation().x() += dX;
		break;
	// Camera Rotation
	case 't': case 'T':
		cam->camera.transform.linear() = getRotationXYZ(getEulerXYZ(cam->camera.transform.rotation()) + Eigen::Vector3f(+dA/180.0f*PI, 0, 0));
		break;
	case 'g': case 'G':
		cam->camera.transform.linear() = getRotationXYZ(getEulerXYZ(cam->camera.transform.rotation()) + Eigen::Vector3f(-dA/180.0f*PI, 0, 0));
		break;
	case 'h': case 'H':
		cam->camera.transform.linear() = getRotationXYZ(getEulerXYZ(cam->camera.transform.rotation()) + Eigen::Vector3f(0, +dA/180.0f*PI, 0));
		break;
	case 'f': case 'F':
		cam->camera.transform.linear() = getRotationXYZ(getEulerXYZ(cam->camera.transform.rotation()) + Eigen::Vector3f(0, -dA/180.0f*PI, 0));
		break;
	case 'r': case 'R':
		cam->camera.transform.linear() = getRotationXYZ(getEulerXYZ(cam->camera.transform.rotation()) + Eigen::Vector3f(0, 0, +dA/180.0f*PI));
		break;
	case 'z': case 'Z':
		cam->camera.transform.linear() = getRotationXYZ(getEulerXYZ(cam->camera.transform.rotation()) + Eigen::Vector3f(0, 0, -dA/180.0f*PI));
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
	TrackingState *state = &host->m_globalState;

	// Render all once at start, mostly to intitialize resources
	for (int m = 0; m < host->m_cameraFrames.size(); m++)
	{
		if (host->m_cameraFrames[m] != NULL)
			host->m_cameraFrames[m]->AssureInit();
	}

	state->testing.targetPose = Eigen::Isometry3f::Identity();
	state->testing.GT = Eigen::Isometry3f::Identity();

	int index = 0;
	while (host->runTestThread)
	{
//		if (updateTestThread)
		{
			updateTestThread = false;
			index = 0;

			// ----- Generate marker pose -----

			auto genPoseInCamView = [](Camera *camera, float min, float max)
			{ // Generate pose facing camera withing range min,max
				const float fov = 1.0f, edge = 1.5f, facing = 30, frontal = 1.8f;
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
			if (state->phase == PHASE_Calibration_Intrinsic || state->phase == PHASE_Calibration_Extrinsic)
			{
				if (state->phase == PHASE_Calibration_Intrinsic || (state->phase == PHASE_Calibration_Extrinsic && state->calibration.relations.size() <= focusOnCamera))
				{ // Marker in front of cameras
					Camera *camera = &state->cameras[focusOnCamera].testing.camera;
					// Generate initial pose
					if (state->testing.targetPose.translation().norm() == 0)
						state->testing.targetPose = state->testing.GT = genPoseInCamView(camera, 20, 60);
					// Generate new target if target is reached
					if ((state->testing.targetPose.translation() - state->testing.GT.translation()).norm() < 1)
						state->testing.targetPose = genPoseInCamView(camera, 20, 60);
				}
				else if (state->phase == PHASE_Calibration_Extrinsic)
				{ // Marker facing two cameras
					Camera *cameraA = &state->cameras[state->calibration.relations[focusOnCamera].camA].testing.camera;
					Camera *cameraB = &state->cameras[state->calibration.relations[focusOnCamera].camB].testing.camera;
					// Generate initial pose
					if (state->testing.targetPose.translation().norm() == 0)
						state->testing.targetPose = state->testing.GT = genPoseInCamViews(cameraA, cameraB, 100, 200);
					// Generate new target if target is reached
					if ((state->testing.targetPose.translation() - state->testing.GT.translation()).norm() < 1)
						state->testing.targetPose = genPoseInCamViews(cameraA, cameraB, 100, 200);
				}
				// Calculate direction
				Eigen::Vector3f tDiff = state->testing.targetPose.translation() - state->testing.GT.translation();
				Eigen::Matrix3f rDiff = state->testing.targetPose.rotation() * state->testing.GT.rotation().transpose();
				Eigen::AngleAxisf rDiffAx = Eigen::AngleAxisf(rDiff);
				// Lerp GT pose to target
				const float cmPerSec = 50.0f;
				float lerp = std::min(1.0f, cmPerSec/host->m_config.mode.cameraFramerate / tDiff.norm());
				state->testing.GT.translation() += tDiff * lerp;
				rDiffAx.angle() *= lerp;
				state->testing.GT.linear() = rDiffAx * state->testing.GT.linear();
			}
			else if (state->phase == PHASE_Calibration_Room)
			{ // Marker at origin
				state->testing.GT = Eigen::Isometry3f::Identity();
			}
			else if (state->phase == PHASE_Tracking)
			{ // Moving marker
				// Generate consistent movement
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
				// Apply
				TGT += TD;
				RGT += RD/180.0f*PI;
				state->testing.GT = createModelMatrix(TGT, getRotationZYX(RGT));
			}

			if (state->phase == PHASE_Calibration_Intrinsic || state->phase == PHASE_Calibration_Extrinsic || state->phase == PHASE_Calibration_Room)
			{		
				// ----- Generate calibration marker data -----

				for (int m = 0; m < state->cameras.size(); m++)
				{
					CameraState *camState = &state->cameras[m];

					// Project marker into camera view (simulated test data)
					camState->points2D.clear();
					camState->pointSizes.clear();
					createMarkerProjection(camState->points2D, camState->pointSizes, camState->testing.markerPtsVisible, *state->calibration.markerTemplate2D, camState->testing.camera, state->testing.GT, state->testing.blobPxStdDev);
					
					// Shuffle only if marker is built-in (so a detection algorithm exists)
					if (isMarkerBuiltIn(state->calibration.markerTemplate2D->id))
						std::random_shuffle(camState->points2D.begin(), camState->points2D.end());
				}

				// ----- Single camera pose estimation -----

				HandleCameraState(&host->m_globalState);
				
				// ----- Pose Error Calculation -----

				for (int m = 0; m < state->cameras.size(); m++)
				{
					CameraState *camState = &state->cameras[m];

					for (int p = 0; p < camState->extrinsic.poses.size(); p++)
					{ // Found marker pose using CV, check against GT (ground truth)
						std::pair<float,float> poseError = calculatePoseError(state->testing.GT, camState->testing.camera.transform * camState->extrinsic.poses[p]);
						float mse = camState->extrinsic.posesMSE[0];
						wxLogMessage(L"Cam %d, Pose %d: MSE %.4fmpx, Error (%.4fmm, %.4f\u00B0)", m, p, mse*1000, poseError.first*10, poseError.second);
					}

					// Add origin for orientation
					//camState->extrinsic.poses.push_back(Eigen::Isometry3f::Identity());
				}
			}
			else if (state->phase == PHASE_Tracking)
			{
				// ----- Generate 3D Marker Data -----

				// For testing of triangulation only
				std::vector<int> visibleCount;
				visibleCount.resize(state->tracking.markerTemplate3D->points.size());

				if (state->tracking.markerTemplate3D->id >= 0) break; // Can't simulate, don't have DefMarker for it
				DefMarker *markerDef = &host->m_config.testing.trackingMarkers[-state->tracking.markerTemplate3D->id-1];

				for (int m = 0; m < state->cameras.size(); m++)
				{
					CameraState *camState = &state->cameras[m];

					// Project marker into camera view (simulated test data)
					camState->points2D.clear();
					camState->pointSizes.clear();
					createMarkerProjection(camState->points2D, camState->pointSizes, camState->testing.markerPtsVisible, *markerDef, camState->testing.camera, state->testing.GT, state->testing.blobPxStdDev);

					// For testing only, accumulate by how many cameras a point is seen to determine if it is useful for triangulation
					for (int i = 0; i < visibleCount.size(); i++)
						visibleCount[i] += (int)camState->testing.markerPtsVisible[i];
					
					// Shuffle points around
					std::random_shuffle(camState->points2D.begin(), camState->points2D.end());
				}

				// ----- Triangulation and marker detection -----

				HandleCameraState(&host->m_globalState);

				// ----- Pose Error Calculation and failure analysis -----

				// Add 3D rays to shared list for visualization
				state->testing.rays3D.clear();
				for (int m = 0; m < state->cameras.size(); m++)
				{
					CameraState *camState = &state->cameras[m];
					state->testing.rays3D.insert(state->testing.rays3D.end(), camState->tracking.rays3D.begin(), camState->tracking.rays3D.end());
				}

				// Gather points which could have been triangulated
				std::bitset<MAX_MARKER_POINTS> triangulationMask;
				for (int i = 0; i < visibleCount.size(); i++)
					triangulationMask.set(i, visibleCount[i] >= 2);
				state->testing.triangulatedPoints3D.clear();
				transformMarkerPoints(state->testing.triangulatedPoints3D, triangulationMask, *state->tracking.markerTemplate3D, state->testing.GT);

				// Handle detected pose
				bool wrongPose = false;
				if (state->tracking.poses3D.size() > 0)
				{ // Determine pose error
					for (int i = 0; i < state->tracking.poses3D.size(); i++)
					{
						std::pair<float,float> poseError = calculatePoseError(state->testing.GT, state->tracking.poses3D[i]);
						std::pair<float,int> poseMSE = state->tracking.posesMSE[i];
						wxLogMessage(L"Pose %d, Points: %d, MSE: %.4fmm, Error: (%.4fmm, %.4f\u00B0)", i, poseMSE.second, poseMSE.first*10, poseError.first*10, poseError.second);
						if (poseError.first > 1 || poseError.second > 2)
						{ // Pose is wrong
							wxLogMessage("--> WRONG POSE: %d reference points", (int)state->testing.triangulatedPoints3D.size());
							wrongPose = true;
						}
					}
				}
				else
				{ // Pose is missing
					wxLogMessage("--> NO POSE: %d reference points!", (int)state->testing.triangulatedPoints3D.size());
					wrongPose = true;
				}

				static int times = 0;
				if (wrongPose && times++ < 10)
				{ // Debug circumstances using ground truth in case pose is wrong (or missing)
					analyzeTrackingAlgorithm(visibleCount, triangulationMask, state->tracking.points3D, *state->tracking.markerTemplate3D, state->testing.GT);
					// Interrupt a few times to allow for debugging
					if (times < 5)
						wxMessageOutput::Get()->Printf (wxT("Failed to detect pose!"));
				}

				// Add origin for orientation
				//state->tracking.poses3D.push_back(Eigen::Isometry3f::Identity());
			}

			// ----- Visualization -----

			for (int m = 0; m < host->m_cameraFrames.size(); m++)
			{
				if (host->m_cameraFrames[m] != NULL)
					host->m_cameraFrames[m]->Repaint();
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds((1000/host->m_config.mode.cameraFramerate)-1));
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

	if (wxGetApp().m_state == STATE_Connected)
	{
		CameraState *camState = &wxGetApp().m_globalState.cameras[0];

		float aspect = (float)wxGetApp().m_config.mode.cameraResolutionY/wxGetApp().m_config.mode.cameraResolutionX;

		// Update list of blobs
		camState->points2D.clear();
		camState->pointSizes.clear();
		camState->points2D.reserve(blobCount);
		camState->pointSizes.reserve(blobCount);
		for (int i = 0; i < blobCount; i++)
		{
			int pos = headerSize+i*6;
			uint16_t *blobData = (uint16_t*)&data[pos];
			float x = (float)((double)blobData[0] / 65536.0);
			float y = (float)((double)blobData[1] / 65536.0 * aspect);
			float s = (float)data[pos+4]/2.0f;
			camState->points2D.push_back(Eigen::Vector2f(x,y));
			camState->pointSizes.push_back(s);
		}

		HandleCameraState(&wxGetApp().m_globalState);

		if (wxGetApp().m_cameraFrames.size() > 0 && wxGetApp().m_cameraFrames[0] != NULL)
			wxGetApp().m_cameraFrames[0]->Repaint();
	}
	else
	{
		wxLogMessage("Failed to receive blobs as state is not initialized yet!");
	}

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