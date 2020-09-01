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
	ID_Test = wxID_HIGHEST + 2,
	ID_Marker = wxID_HIGHEST + 3,
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
	for (int i = 0; i < m_config.testing.markerDefinitionFiles.size(); i++)
	{
		if (!parseMarkerDataFile(m_config.testing.markerDefinitionFiles[i], &m_markerData))
		{
			wxMessageOutput::Get()->Printf (wxT("Failed to read Marker '%s'!"), m_config.testing.markerDefinitionFiles[i]);
		}
	}
	/*for (int i = 0; i < m_markerData.size(); i++)
	{
		wxMessageOutput::Get()->Printf (wxT("Marker '%s' has %d points!"), m_markerData[i].label, (int)m_markerData[i].pts.size());
	}*/

	if (m_config.testing.markerDefinitionFiles.size() > 0)
	{ // Set first as default
		setActiveMarkerData(m_markerData[0]);
	}
	else
	{ // Set some default Marker
		setActiveMarkerData({ "Default 1-Point", { { Eigen::Vector3f( 0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 1.0f), 360.0f } } });
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
void ConfiguratorApp::StartTesting()
{
	Disconnect();
	if (m_state == STATE_Testing) return;
	wxLogMessage("Starting testing phase!");
	// Add testing cameras
	for (int i = 0; i < m_config.testing.cameraDefinitions.size(); i++)
	{
		MarkerDetector cam = {};
		cam.state.testing.cameraGT = {
			m_config.testing.cameraDefinitions[i].pos,
			m_config.testing.cameraDefinitions[i].rot
		};
		std::stringstream dbgPos, dbgRot;
		dbgPos << cam.state.testing.cameraGT.pos.transpose();
		dbgRot << getEulerXYZ(cam.state.testing.cameraGT.rot).transpose();
		wxLogMessage("Cam %s Pos: %s, Rot: %s", m_config.testing.cameraDefinitions[i].label, dbgPos.str(), dbgRot.str());
		cam.frame = new CameraFrame(m_config.testing.cameraDefinitions[i].label);
		m_markerDetectors.push_back(cam);
	}
	// Start test data thread
	runTestThread = true;
	if (testThread == NULL) testThread = new std::thread(ThreadSendTestData);
	m_state = STATE_Testing;
}
void ConfiguratorApp::StopTesting()
{
	if (m_state != STATE_Testing) return;
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
EVT_MENU(ID_Test, ConfiguratorFrame::OnStartTesting)
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
	menu->Append(ID_Test, "&Test Marker", "Start testing phase of selected marker.");
	wxMenu *markerTestMenu = new wxMenu();
	menu->AppendSubMenu(markerTestMenu, "&Markers", "Select a specific marker shape to test.");
	menu->AppendSeparator();
	menu->Append(wxID_ABOUT);
	menu->AppendSeparator();
	menu->Append(wxID_EXIT);

	// Fill marker selection
	for (int i = 0; i < wxGetApp().m_markerData.size(); i++)
	{
		DefMarker *marker = &wxGetApp().m_markerData[i];
		std::stringstream menuLabel;
		menuLabel << "&" << marker->label;
		markerTestMenu->Append(ID_Marker+i, menuLabel.str(), "Test this marker shape.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+i);
		wxLogMessage("Read Marker '%s' with %d points!", marker->label, (int)marker->pts.size());
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
	wxGetApp().StartTesting();
}
void ConfiguratorFrame::OnStopTesting(wxCommandEvent &event)
{
	wxGetApp().StopTesting();
}
void ConfiguratorFrame::OnSelectMarker(wxCommandEvent &event)
{
	int markerID = event.GetId()-ID_Marker;
	DefMarker *marker = &wxGetApp().m_markerData.at(markerID);
	wxLogMessage("Set testing marker to '%s' with %d points!", marker->label, (int)marker->pts.size());
	setActiveMarkerData(*marker);
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
		initPoseInference(camWidth, camHeight);

		GLInit = true;
	}
	return GLInit;
}
static void assureGLClean()
{
	// Clean pose inference
	cleanPoseInference();
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
//	m_canvas->Refresh(false);
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
//	visualizePoses(camState->camera, camState->points2D, camState->m_markers, camState->poses);
	visualizeMarkers(camState->camera, camState->points2D, wxGetApp().rays3D, wxGetApp().points3D, wxGetApp().nonconflictedCount, wxGetApp().testing.triangulatedPoints3D, wxGetApp().poses3D);

	//glFlush();
	m_canvas->SwapBuffers();
}
void CameraFrame::OnPaint(wxPaintEvent &event)
{
	wxPaintDC dc(m_canvas);
//	Render();
}

static double TGT[3] { 0, 0, 100 };
static double RGT[3] { 0, 0, 0 };
static double TD[3] { 0, 0, 0 };
static double RD[3] { 0, 0, 0 };
const float dA = 5, dX = 10;
static bool updateTestThread = true;
void CameraFrame::OnKeyDown(wxKeyEvent &event)
{
	CameraState *camState = wxGetApp().GetCameraState(this);
	switch (event.GetKeyCode())
	{
	// Target Marker Rotation
	case 'q': case 'Q':
		RGT[2] += dA/180.0f*PI;
		break;
	case 'e': case 'E':
		RGT[2] -= dA/180.0f*PI;
		break;
	case 'w': case 'W':
		RGT[0] -= dA/180.0f*PI;
		break;
	case 's': case 'S':
		RGT[0] += dA/180.0f*PI;
		break;
	case 'a': case 'A':
		RGT[1] -= dA/180.0f*PI;
		break;
	case 'd': case 'D':
		RGT[1] += dA/180.0f*PI;
		break;
	// Target Marker Position
	case WXK_UP:
		TGT[1] += dX;
		break;
	case WXK_DOWN:
		TGT[1] -= dX;
		break;
	case WXK_LEFT:
		TGT[0] -= dX;
		break;
	case WXK_RIGHT:
		TGT[0] += dX;
		break;
	case WXK_PAGEUP:
		TGT[2] += dX;
		break;
	case WXK_PAGEDOWN:
		TGT[2] -= dX;
		break;
	// Camera Position
	case 'i': case 'I':
		camState->camera.pos.z() -= dX;
		break;
	case 'k': case 'K':
		camState->camera.pos.z() += dX;
		break;
	case 'j': case 'J':
		camState->camera.pos.x() -= dX;
		break;
	case 'l': case 'L':
		camState->camera.pos.x() += dX;
		break;
	// Camera Rotation
	case 't': case 'T':
		camState->camera.rot = getRotationXYZ(getEulerXYZ(camState->camera.rot) + Eigen::Vector3f(+dA/180.0f*PI, 0, 0));
		break;
	case 'g': case 'G':
		camState->camera.rot = getRotationXYZ(getEulerXYZ(camState->camera.rot) + Eigen::Vector3f(-dA/180.0f*PI, 0, 0));
		break;
	case 'h': case 'H':
		camState->camera.rot = getRotationXYZ(getEulerXYZ(camState->camera.rot) + Eigen::Vector3f(0, +dA/180.0f*PI, 0));
		break;
	case 'f': case 'F':
		camState->camera.rot = getRotationXYZ(getEulerXYZ(camState->camera.rot) + Eigen::Vector3f(0, -dA/180.0f*PI, 0));
		break;
	case 'r': case 'R':
		camState->camera.rot = getRotationXYZ(getEulerXYZ(camState->camera.rot) + Eigen::Vector3f(0, 0, +dA/180.0f*PI));
		break;
	case 'z': case 'Z':
		camState->camera.rot = getRotationXYZ(getEulerXYZ(camState->camera.rot) + Eigen::Vector3f(0, 0, -dA/180.0f*PI));
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
	// Render all once at start, mostly to intitialize resources
	for (int m = 0; m < wxGetApp().m_markerDetectors.size(); m++)
	{
		MarkerDetector *markerDetector = &wxGetApp().m_markerDetectors[m];
		CameraState *camState = &markerDetector->state;
		if (markerDetector->frame != NULL)
			markerDetector->frame->Repaint();
	}

	int index = 0;
	while (wxGetApp().runTestThread)
	{
//		if (++index >= (1000/testFrameRate)-1)
//		if (updateTestThread)
		{
			updateTestThread = false;
			index = 0;

			// Reset 3d ray visualization
			wxGetApp().rays3D.clear();

			// Create random Ground Truth transform

			// Generate completely random poses
			/*TGT[2] = (rand()%10000 / 10000.0f) * 200 + 20;
			TGT[0] = (rand()%10000 / 10000.0f) * TGT[2]/2 - TGT[2]/4;
			TGT[1] = (rand()%10000 / 10000.0f) * TGT[2]/2 - TGT[2]/4;
			RGT[0] = (rand()%10000 / 10000.0f) * 100 - 50;
			RGT[1] = (rand()%10000 / 10000.0f) * 100 - 50;
			RGT[2] = (rand()%10000 / 10000.0f) * 100 - 50;*/

			// Generate consistently moving pose
			TD[2] += (rand()%10000 / 10000.0f) * 0.1 - 0.05;
			TD[0] += (rand()%10000 / 10000.0f) * 0.1 - 0.05;
			TD[1] += (rand()%10000 / 10000.0f) * 0.1 - 0.05;
			RD[0] += ((rand()%10000 / 10000.0f) * 2 - 1);
			RD[1] += ((rand()%10000 / 10000.0f) * 2 - 1);
			RD[2] += ((rand()%10000 / 10000.0f) * 2 - 1);
			// Dampen movement
			TD[0] *= 0.9;
			TD[1] *= 0.9;
			TD[2] *= 0.9;
			RD[0] *= 0.95;
			RD[1] *= 0.95;
			RD[2] *= 0.95;
			// Apply movement
			TGT[0] += TD[0];
			TGT[1] += TD[1];
			TGT[2] += TD[2];
			RGT[0] += RD[0]/180.0f*PI;
			RGT[1] += RD[1]/180.0f*PI;
			RGT[2] += RD[2]/180.0f*PI;

			// Transform to Eigen
			Eigen::Vector3f tGT(TGT[0], TGT[1], TGT[2]);
			Eigen::Matrix3f rGT = getRotationZYX(Eigen::Vector3f(RGT[0], RGT[1], RGT[2]));

			// For testing of triangulation only
			std::vector<int> visibleCount;
			visibleCount.resize(getExpectedBlobCount());

			for (int m = 0; m < wxGetApp().m_markerDetectors.size(); m++)
			{
				MarkerDetector *markerDetector = &wxGetApp().m_markerDetectors[m];
				CameraState *camState = &markerDetector->state;

				// Add origin for orientation
				camState->poses.clear();
				camState->poses.push_back({
					Eigen::Vector3f(0, 0, 0),
					Eigen::Matrix3f::Identity(),
				});

				// Project marker into camera view (simulated test data)
				camState->points2D.clear();
				createMarkerProjection(camState->points2D, camState->testing.markerPtsVisible, camState->testing.cameraGT, tGT, rGT, 1.0f, 0.02f);

				// For testing only, accumulate by how many cameras a point is seen to determine if it is useful for triangulation
				for (int i = 0; i < camState->testing.markerPtsVisible.size(); i++)
					visibleCount[i] += (int)camState->testing.markerPtsVisible[i];

				// Assume perfect knowledge of camera position (TODO: infer using calibration)
				camState->camera = camState->testing.cameraGT;

				// Assume perfect order of image points
				std::random_shuffle(camState->points2D.begin(), camState->points2D.end());

				{ // Single camera pose estimation using OpenCV
					// This will only be used for calibration later on

					// Assume all points visible belong to singular marker (TODO: Marker detection and point ordering)
	//				findMarkerCandidates(camState->points2D, camState->m_markers, camState->m_freeBlobs);

					// Infer pose of generic marker under above assumptions
					if (camState->points2D.size() >= getExpectedBlobCount())
					{
						Pose pose;
						inferMarkerPoseGeneric(camState->points2D, camState->camera, pose);
						camState->poses.push_back(pose);
					}

					// Infer pose of detected markers
		//			inferMarkerPoses(camState->m_markers, camState->camera, camState->poses);

					if (camState->poses.size() > 1)
					{ // Found marker pose using CV, check against GT (ground truth)
						Eigen::Vector3f tDiff = camState->poses[1].trans-tGT;
						Eigen::Matrix3f rDiff = rGT * camState->poses[1].rot.transpose();
						wxLogMessage(L"Cam %d: CV Translation Error: %fcm -- Angle Error: %f\u00B0", m, tDiff.norm(), Eigen::AngleAxisf(rDiff).angle()/PI*180);
					}
				}

				// Create 3D Rays for triangulation assuming calibrated camera
				camState->rays3D.clear();
				castRays(camState->points2D, camState->camera, camState->rays3D);
				// Add to shared list for visualization only
				wxGetApp().rays3D.insert(wxGetApp().rays3D.end(), camState->rays3D.begin(), camState->rays3D.end());
			}

			// For visualization, add points which could be triangulated to a list
			std::bitset<MAX_MARKER_POINTS> triangulationMask;
			for (int i = 0; i < visibleCount.size(); i++)
				triangulationMask.set(i, visibleCount[i] >= 2);
			transformMarkerPoints(wxGetApp().testing.triangulatedPoints3D, triangulationMask, tGT, rGT);

			// Perform triangulation using 3D Rays
			std::vector<std::vector<Ray>*> rayGroups;
			for (int m = 0; m < wxGetApp().m_markerDetectors.size(); m++)
				rayGroups.push_back(&wxGetApp().m_markerDetectors[m].state.rays3D);
			wxGetApp().points3D.clear();
			wxGetApp().conflicts.clear();
			wxGetApp().nonconflictedCount = triangulateRayIntersections(rayGroups, wxGetApp().points3D, wxGetApp().conflicts, 0.01f);

			// Detect markers in triangulated point cloud
			wxGetApp().poses3D.clear();
			detectMarkers3D(wxGetApp().points3D, wxGetApp().conflicts, wxGetApp().nonconflictedCount, wxGetApp().poses3D);

			bool wrongPose = false;
			if (wxGetApp().poses3D.size() > 0)
			{
				Pose *pose = &wxGetApp().poses3D[0];
				Eigen::Vector3f tDiff = pose->trans - tGT;
				Eigen::Matrix3f rDiff = rGT * pose->rot.transpose();
				float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
				wxLogMessage(L"--> Pose Error: Pos %fcm -- Angle %f\u00B0", tError, rError);
				if (tError > 1 || rError > 2) wrongPose = true;
			}
			else wrongPose = true;

			if (wrongPose)
			{
				if (wxGetApp().poses3D.size() == 0)
					wxLogMessage("Pose not detected! Had %d true triangulated marker points!", (int)wxGetApp().testing.triangulatedPoints3D.size());
				else
					wxLogMessage("Pose wrongly detected! Had %d true triangulated marker points!", (int)wxGetApp().testing.triangulatedPoints3D.size());

				// Analyze with knowledge which triangulated points could be used for the marker detection
				std::vector<int> relationMask;
				relationMask.resize(getExpectedBlobCount());
				std::vector<std::pair<int, float>> mk2trMap;
				mk2trMap.resize(getExpectedBlobCount());
				std::vector<PointRelation*> mkRelations;
				Eigen::Isometry3f gt;
				gt.linear() = rGT;
				gt.translation() = tGT;
				for (int i = 0; i < visibleCount.size(); i++)
				{
					int mkPtA = i;

					Eigen::Vector3f trPos = gt * marker3D.markerTemplate.pts[i].pos;
					float trError = 9999.0f;
					int trPt = -1;
					for (int j = 0; j < wxGetApp().points3D.size(); j++)
					{
						float error = (wxGetApp().points3D[j].pos.head<3>() - trPos).norm();
						if (error < trError)
						{
							trError = error;
							trPt = j;
						}
					}

					if (triangulationMask.test(mkPtA))
					{
						mk2trMap[i].first = trPt;
						mk2trMap[i].second = trError;
						wxLogMessage("Found matching Marker point %d => %d within %f cm!", i, trPt, trError);

						for (int j = 0; j < marker3D.pointRelation[mkPtA].size(); j++)
						{
							int relInd = marker3D.pointRelation[mkPtA][j];
							PointRelation *rel = &marker3D.relationDist[relInd];
							int mkPtB = rel->pt1 == mkPtA? rel->pt2 : rel->pt1;
							if (mkPtB > mkPtA && triangulationMask.test(mkPtB))
							{ // rel could have been inferred in marker, as both points ARE in the triangulated Point Cloud
								float minError = 3 * (0.01f + 0.01f);
								auto mkRelRange = std::equal_range(marker3D.relationDist.begin(), marker3D.relationDist.end(), rel->distance, ErrorRangeComp(minError));
								wxLogMessage("--> Relation (%d - %d) is in triangulated point cloud with minimum of %d candidates!", mkPtA, mkPtB, (int)std::distance(mkRelRange.first, mkRelRange.second));
								mkRelations.push_back(rel);
								if (relationMask[mkPtA] != 0 || relationMask[mkPtB] != 0)
								{ // Found a triple of triangulated points, so a candidate SHOULD have been found
									//wxLogMessage("!!!!!! ----> Triple Detected!");
								}
								relationMask[mkPtA]++;
								relationMask[mkPtB]++;
							}
						}
					}
					else if (trError < 0.1f)
					{ // Even though it should NOT be triangulated, there is a fake point in it
						mk2trMap[i].first = trPt;
						mk2trMap[i].second = trError;
						wxLogMessage("!!!!!! Marker point %d has a fake triangulated point %d within %f cm!", i, trPt, trError);
					}
				}

				for (int i = 0; i < mkRelations.size(); i++)
				{
					if (relationMask[mkRelations[i]->pt1] > 1)
					{
						int mkPtJ = mkRelations[i]->pt1;
						int mkPtB = mkRelations[i]->pt2;
						for (int j = 0; j < mkRelations.size(); j++)
						{
							if (mkRelations[j]->pt1 == mkPtJ || mkRelations[j]->pt2 == mkPtJ)
							{
								int mkPtA = mkRelations[j]->pt1 == mkPtJ? mkRelations[j]->pt2 : mkRelations[j]->pt1;
								if (mkPtA == mkPtB) continue;
								bool sharedArm = false;
								for (int l = 0; l < marker3D.pointRelation[mkPtA].size(); l++)
									if (sharedArm = (marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt1 == mkPtB || marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt2 == mkPtB))
										break;

								wxLogMessage("Found triple (%d - %d / %f - %f), (%d - %d / %f - %f), (%d - %d / %f - %f)! B-A have shared arm: %s!",
									mkPtB, mk2trMap[mkPtB].first, mk2trMap[mkPtB].second, 3*wxGetApp().points3D[mk2trMap[mkPtB].first].error,
									mkPtJ, mk2trMap[mkPtJ].first, mk2trMap[mkPtJ].second, 3*wxGetApp().points3D[mk2trMap[mkPtJ].first].error,
									mkPtA, mk2trMap[mkPtA].first, mk2trMap[mkPtA].second, 3*wxGetApp().points3D[mk2trMap[mkPtA].first].error,
									sharedArm? "True" : "False");

							}
						}
					}
					else if (relationMask[mkRelations[i]->pt2] > 1)
					{
						int mkPtJ = mkRelations[i]->pt2;
						int mkPtB = mkRelations[i]->pt1;
						for (int j = 0; j < mkRelations.size(); j++)
						{
							if (mkRelations[j]->pt1 == mkPtJ || mkRelations[j]->pt2 == mkPtJ)
							{
								int mkPtA = mkRelations[j]->pt1 == mkPtJ? mkRelations[j]->pt2 : mkRelations[j]->pt1;
								if (mkPtA == mkPtB) continue;
								bool sharedArm = false;
								for (int l = 0; l < marker3D.pointRelation[mkPtA].size(); l++)
									if (sharedArm = (marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt1 == mkPtB || marker3D.relationDist[marker3D.pointRelation[mkPtA][l]].pt2 == mkPtB))
										break;

								wxLogMessage("Found triple (%d - %d / %f - %f), (%d - %d / %f - %f), (%d - %d / %f - %f)! B-A have shared arm: %s!",
									mkPtB, mk2trMap[mkPtB].first, mk2trMap[mkPtB].second, 3*wxGetApp().points3D[mk2trMap[mkPtB].first].error,
									mkPtJ, mk2trMap[mkPtJ].first, mk2trMap[mkPtJ].second, 3*wxGetApp().points3D[mk2trMap[mkPtJ].first].error,
									mkPtA, mk2trMap[mkPtA].first, mk2trMap[mkPtA].second, 3*wxGetApp().points3D[mk2trMap[mkPtA].first].error,
									sharedArm? "True" : "False");

							}
						}
					}

				}

				// Interrupt
				static int times = 0;
				if (times++ < 5) wxMessageOutput::Get()->Printf (wxT("Failed to detect pose!"));
			}

			wxGetApp().poses3D.push_back({
				Eigen::Vector3f(0, 0, 0),
				Eigen::Matrix3f::Identity(),
			});

			// Start visualization
			for (int m = 0; m < wxGetApp().m_markerDetectors.size(); m++)
			{
				MarkerDetector *markerDetector = &wxGetApp().m_markerDetectors[m];
				CameraState *camState = &markerDetector->state;
				if (markerDetector->frame != NULL)
					markerDetector->frame->Repaint();
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