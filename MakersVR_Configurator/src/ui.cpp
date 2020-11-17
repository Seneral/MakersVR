/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "GL/glew.h"

#include "ui.hpp"

#include "visualization.hpp"

enum CustomEvents
{
	ID_Connect = wxID_HIGHEST + 1,
	ID_Disconnect,
	ID_StartStreaming,
	ID_StopStreaming,
	ID_StartTesting,
	ID_StopTesting,
	ID_Marker, // Always has to be last since higher IDs are used to identify marker
};

/* Functions */

static void HandleInput(wxKeyEvent &event, CameraState *cam);

static bool assureGLInit();

/* Testing and Debug variables */

// ----------------------------------------------------------------------------
// ConfiguratorApp
// ----------------------------------------------------------------------------

wxIMPLEMENT_APP(ConfiguratorApp);

static void OnUpdateCam(int n)
{
	if (wxGetApp().m_cameraFrames.size() > n && wxGetApp().m_cameraFrames[n] != NULL)
		wxGetApp().m_cameraFrames[n]->Repaint();
}

bool ConfiguratorApp::OnInit()
{
	if (!ConfiguratorInit(m_state))
		return false;
	// Setup callback
	m_state.OnUpdateCam = OnUpdateCam;
	// Open main frame
	m_frame = new ConfiguratorFrame();
	return true;
}

int ConfiguratorApp::OnExit()
{
	// Cleanup GL
	cleanVisualization();
	if (m_glContext)
		delete m_glContext;
	// Exit
	ConfiguratorExit(m_state);
	return 0;
}

wxGLContext *ConfiguratorApp::GetContext(wxGLCanvas *canvas)
{
	if (!m_glContext) m_glContext = new wxGLContext(canvas);
	return m_glContext;
}

CameraState *ConfiguratorApp::GetCameraState(CameraFrame *frame)
{
	for (int i = 0; i < m_state.control.cameras.size(); i++)
	{
		if (m_cameraFrames[i] == frame)
			return &m_state.control.cameras[i];
	}
	return NULL;
}

void ConfiguratorApp::SetupUI()
{
	if (m_state.control.cameras.size() == 0) return;
	for (int i = 0; i < m_state.control.cameras.size(); i++)
	{
		CameraState *cam = &m_state.control.cameras[i];
		m_cameraFrames.push_back(new CameraFrame("MarkerDetector " + std::to_string(i) + ": " + cam->camera.label, (float)cam->camera.height/cam->camera.width));

		if (m_state.control.testing.isTesting)
		{ // Debug
			Camera *tCam = &cam->testing.camera;
			Camera *cCam = &cam->camera;
			wxLogMessage("Cam %d labeled %s", i, m_state.config.testing.cameraDefinitions[i].label);
			wxLogMessage("Cam %d testing intrinsics: FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", i, tCam->fovH, tCam->fovV,
				tCam->distortion.k1, tCam->distortion.k2, tCam->distortion.p1, tCam->distortion.p2, tCam->distortion.k3);
			if (cCam->fovH != 0)
			{
				wxLogMessage("Cam %d calibrated intrinsics: FoV (%f, %f), Distortions (%f, %f, %f, %f, %f)", i, cCam->fovH, cCam->fovV,
					cCam->distortion.k1, cCam->distortion.k2, cCam->distortion.p1, cCam->distortion.p2, cCam->distortion.k3);
			}
			Eigen::Vector3f posGT = tCam->transform.translation();
			Eigen::Vector3f rotGT = getEulerXYZ(tCam->transform.rotation()) / PI * 180;
			wxLogMessage("Cam %d testing transform: Pos/cm (%.2f, %.2f, %.2f), Rot/\u00B0 (%.2f, %.2f, %.2f)", i,
				posGT.x(), posGT.y(), posGT.z(), rotGT.x(), rotGT.y(), rotGT.z());
			if (cCam->transform.translation().sum() != 0)
			{
				Eigen::Vector3f posCB = cCam->transform.translation();
				Eigen::Vector3f rotCB = getEulerXYZ(cCam->transform.rotation()) / PI * 180;
				Eigen::Vector3f tDiff = cCam->transform.translation() - tCam->transform.translation();
				Eigen::Matrix3f rDiff = tCam->transform.rotation() * cCam->transform.rotation().transpose();
				float tError = tDiff.norm(), rError = Eigen::AngleAxisf(rDiff).angle()/PI*180;
				wxLogMessage("Cam %d calibrated transform: Pos/cm (%.2f, %.2f, %.2f), Rot/\u00B0 (%.2f, %.2f, %.2f), Error: (%.4fmm, %.4f\u00B0)", i,
					posCB.x(), posCB.y(), posCB.z(), rotCB.x(), rotCB.y(), rotCB.z(), tError*10, rError);
			}
		}
	}
	m_frame->phaseSelector->SetSelection(0);
	wxSizer *sizer = m_frame->mainPanel->GetSizer();
	sizer->Show((size_t)0, false);
	sizer->Show((size_t)1, true);
	sizer->Layout();
}

void ConfiguratorApp::ResetUI()
{
	for (int i = 0; i < m_cameraFrames.size(); i++)
	{
		if (m_cameraFrames[i] != NULL)
			m_cameraFrames[i]->Destroy();
	}
	m_cameraFrames.clear();
	wxSizer *sizer = m_frame->mainPanel->GetSizer();
	sizer->Show((size_t)0, true);
	sizer->Show((size_t)1, false);
	sizer->Layout();
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
		StopStreaming(m_state);
		StopTesting(m_state);
		ResetUI();
	}
}

// ----------------------------------------------------------------------------
// ConfiguratorFrame
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(ConfiguratorFrame, wxFrame)
// Configurator
EVT_MENU(wxID_ABOUT, ConfiguratorFrame::OnAbout)
EVT_MENU(wxID_EXIT, ConfiguratorFrame::OnExit)
// Device
EVT_MENU(ID_Connect, ConfiguratorFrame::OnConnect)
EVT_MENU(ID_Disconnect, ConfiguratorFrame::OnDisconnect)
EVT_MENU(ID_StartStreaming, ConfiguratorFrame::OnStartStreaming)
EVT_MENU(ID_StopStreaming, ConfiguratorFrame::OnStopStreaming)
// Testing
EVT_MENU(ID_StartTesting, ConfiguratorFrame::OnStartTesting)
EVT_MENU(ID_StopTesting, ConfiguratorFrame::OnStopTesting)
// General
EVT_CLOSE(ConfiguratorFrame::OnClose)
wxEND_EVENT_TABLE()

ConfiguratorFrame::ConfiguratorFrame()
	: wxFrame(NULL, wxID_ANY, "MakersVR Configurator", wxPoint(20, 20), wxSize(480, 360))
{
	// Main Layout
    mainPanel = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxWANTS_CHARS);
	wxBoxSizer *mainSizer = new wxBoxSizer(wxVERTICAL);
	mainPanel->SetSizer(mainSizer);
	// Keyboard Events
	mainPanel->Bind(wxEVT_CHAR_HOOK, &ConfiguratorFrame::OnKeyDown, this);
	
	// Setup control area
	wxPanel *setupControlsPanel = new wxPanel(mainPanel);
	mainSizer->Add(setupControlsPanel, 0);
	wxBoxSizer *setupControlsSizer = new wxBoxSizer(wxHORIZONTAL);
	setupControlsPanel->SetSizer(setupControlsSizer);
	// Setup control buttons
	// Width
	setupControlsSizer->AddSpacer(10);
	wxStaticText *widthLabel = new wxStaticText(setupControlsPanel, wxID_ANY, "Width");
	setupControlsSizer->Add(widthLabel, wxSizerFlags().HorzBorder().CenterVertical());
	widthField = new wxSpinCtrl(setupControlsPanel);
	widthField->SetRange(64, 5000);
	widthField->SetValue(wxGetApp().m_state.config.mode.cameraResolutionX);
	widthField->Bind(wxEVT_SPINCTRL, &ConfiguratorFrame::OnChangeSetup, this);
	setupControlsSizer->Add(widthField, wxSizerFlags().Expand());
	// Height
	setupControlsSizer->AddSpacer(10);
	wxStaticText *heightLabel = new wxStaticText(setupControlsPanel, wxID_ANY, "Height");
	setupControlsSizer->Add(heightLabel, wxSizerFlags().HorzBorder().CenterVertical());
	heightField = new wxSpinCtrl(setupControlsPanel);
	heightField->SetRange(64, 5000);
	heightField->SetValue(wxGetApp().m_state.config.mode.cameraResolutionY);
	heightField->Bind(wxEVT_SPINCTRL, &ConfiguratorFrame::OnChangeSetup, this);
	setupControlsSizer->Add(heightField, wxSizerFlags().Expand());
	// FPS
	setupControlsSizer->AddSpacer(10);
	wxStaticText *fpsLabel = new wxStaticText(setupControlsPanel, wxID_ANY, "FPS");
	setupControlsSizer->Add(fpsLabel, wxSizerFlags().HorzBorder().CenterVertical());
	fpsField = new wxSpinCtrl(setupControlsPanel);
	fpsField->SetRange(1, 200);
	fpsField->SetValue(wxGetApp().m_state.config.mode.cameraFramerate);
	fpsField->Bind(wxEVT_SPINCTRL, &ConfiguratorFrame::OnChangeSetup, this);
	setupControlsSizer->Add(fpsField, wxSizerFlags().Expand());
	// Shutter Speed
	setupControlsSizer->AddSpacer(10);
	wxStaticText *ssLabel = new wxStaticText(setupControlsPanel, wxID_ANY, "Shutter Speed");
	setupControlsSizer->Add(ssLabel, wxSizerFlags().HorzBorder().CenterVertical());
	ssField = new wxSpinCtrl(setupControlsPanel);
	ssField->SetRange(1, 5000);
	ssField->SetValue(wxGetApp().m_state.config.mode.cameraShutterSpeed);
	ssField->Bind(wxEVT_SPINCTRL, &ConfiguratorFrame::OnChangeSetup, this);
	setupControlsSizer->Add(ssField, wxSizerFlags().Expand());

	// Phase control area
	wxPanel *phaseControlsPanel = new wxPanel(mainPanel);
	mainSizer->Add(phaseControlsPanel, 0);
	mainSizer->Hide(phaseControlsPanel);
	wxBoxSizer *phaseControlsSizer = new wxBoxSizer(wxHORIZONTAL);
	phaseControlsPanel->SetSizer(phaseControlsSizer);
	// Phase Dropdown
	phaseSelector = new wxChoice(phaseControlsPanel, wxID_ANY);
	phaseSelector->AppendString("Idle");
	phaseSelector->AppendString("Intrinsic Calibration");
	phaseSelector->AppendString("Extrinsic Calibration");
	phaseSelector->AppendString("Marker Calibration");
	phaseSelector->AppendString("Tracking");
	phaseSelector->SetSelection(0);
	phaseSelector->Bind(wxEVT_CHOICE, &ConfiguratorFrame::OnSelectPhase, this);
	phaseControlsSizer->Add(phaseSelector, 1);
	// Phase control buttons
	buttonAccept = new wxButton(phaseControlsPanel, wxID_ANY, "Accept");
	buttonAccept->Bind(wxEVT_BUTTON, &ConfiguratorFrame::OnAcceptPhase, this);
	phaseControlsSizer->Add(buttonAccept, 1);
	wxButton *buttonDiscard = new wxButton(phaseControlsPanel, wxID_ANY, "Discard");
	buttonDiscard->Bind(wxEVT_BUTTON, &ConfiguratorFrame::OnDiscardPhase, this);
	phaseControlsSizer->Add(buttonDiscard, 1);

	// Debug field
	wxTextCtrl *logText = new wxTextCtrl(mainPanel, wxID_ANY, "MakersVR Configurator\n", wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_WORDWRAP | wxTE_READONLY);
	wxLog::SetActiveTarget(new wxLogTextCtrl(logText));
	mainSizer->Add(logText, 1, wxEXPAND | wxALL);

	// Status bar
	CreateStatusBar();

	// Menu bar
	wxMenuBar *menuBar = new wxMenuBar();
	wxMenu *configMenu = new wxMenu();
	menuBar->Append(configMenu, "Configurator");
	wxMenu *deviceMenu = new wxMenu();
	menuBar->Append(deviceMenu, "Device");
	wxMenu *testMenu = new wxMenu();
	menuBar->Append(testMenu, "Testing");
	SetMenuBar(menuBar);
	// Setup configurator menu dropdown
	configMenu->Append(wxID_ABOUT);
	configMenu->AppendSeparator();
	configMenu->Append(wxID_EXIT);
	// Setup device menu dropdown
	deviceMenu->Append(ID_Connect, "Connect", "Start connecting to a MakersVR device");
	deviceMenu->Append(ID_Disconnect, "Disconnect", "Disconnect from the connected device");
	deviceMenu->AppendSeparator();
	deviceMenu->Append(ID_StartStreaming, "Start Streaming", "Start streaming blobs from connected device");
	deviceMenu->Append(ID_StopStreaming, "Stop Streaming", "Stop streaming blobs from connected device");
	// Setup testing menu dropdown
	testMenu->Append(ID_StartTesting, "Start Testing", "Start the testing phase.");
	testMenu->Append(ID_StopTesting, "Stop Testing", "Stop the testing phase.");
	testMenu->AppendSeparator();
	wxMenu *calibMarkerMenu = new wxMenu();
	testMenu->AppendSubMenu(calibMarkerMenu, "Calibration Markers", "Select the marker shape to use for calibration testing.");
	wxMenu *trackMarkerMenu = new wxMenu();
	testMenu->AppendSubMenu(trackMarkerMenu, "Tracking Markers", "Select the marker shape to use for tracking testing.");

	ControlState *control = &wxGetApp().m_state.control;

	// Fill calibration marker selection (built-in + config)
	for (int i = 0; i < control->calibration.markerTemplates2D.size(); i++)
	{
		DefMarker *calibMarker = &control->calibration.markerTemplates2D[i];
		calibMarkerMenu->Append(ID_Marker+i, calibMarker->label, "Select this calibration marker.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+i);
		wxLogMessage("Read calibration Marker '%s' (%d) with %d points!", calibMarker->label, calibMarker->id, (int)calibMarker->points.size());
	}

	// Fill tracking marker selection (only testing, not calibrated markers)
	int idOffset = control->calibration.markerTemplates2D.size();
	for (int i = 0; i < wxGetApp().m_state.config.testing.trackingMarkers.size(); i++)
	{
		DefMarker *trackMarker = &wxGetApp().m_state.config.testing.trackingMarkers[i];
		trackMarkerMenu->Append(ID_Marker+idOffset+i, trackMarker->label, "Select this tracking marker.");
		Bind(wxEVT_COMMAND_MENU_SELECTED, &ConfiguratorFrame::OnSelectMarker, this, ID_Marker+idOffset+i);
		wxLogMessage("Read test tracking marker '%s' (%d) with %d points!", trackMarker->label, trackMarker->id, (int)trackMarker->points.size());
	}

	Show();
}
void ConfiguratorFrame::OnClose(wxCloseEvent& event)
{
	CommDisconnect(wxGetApp().m_state);
	StopTesting(wxGetApp().m_state);
	wxGetApp().ResetUI();
	Destroy();
}
void ConfiguratorFrame::OnExit(wxCommandEvent &event)
{
	StopStreaming(wxGetApp().m_state);
	StopTesting(wxGetApp().m_state);
	wxGetApp().ResetUI();
	Destroy();
}
void ConfiguratorFrame::OnAbout(wxCommandEvent &event)
{
	wxMessageBox("MakersVR Configurator is used to configure the MakersVR Tracking System",
				 "About MakersVR Configurator", wxOK | wxICON_INFORMATION);
}
void ConfiguratorFrame::OnConnect(wxCommandEvent &event)
{
	CommConnect(wxGetApp().m_state);
}
void ConfiguratorFrame::OnDisconnect(wxCommandEvent &event)
{
	CommDisconnect(wxGetApp().m_state);
}
void ConfiguratorFrame::OnStartStreaming(wxCommandEvent &event)
{
	StartStreaming(wxGetApp().m_state);
	wxGetApp().SetupUI();
}
void ConfiguratorFrame::OnStopStreaming(wxCommandEvent &event)
{
	StopStreaming(wxGetApp().m_state);
	wxGetApp().ResetUI();
}
void ConfiguratorFrame::OnStartTesting(wxCommandEvent &event)
{
	StartTesting(wxGetApp().m_state);
	wxGetApp().SetupUI();
}
void ConfiguratorFrame::OnStopTesting(wxCommandEvent &event)
{
	StopTesting(wxGetApp().m_state);
	wxGetApp().ResetUI();
}
void ConfiguratorFrame::OnSelectMarker(wxCommandEvent &event)
{
	ControlState *control = &wxGetApp().m_state.control;
	int markerID = event.GetId()-ID_Marker;
	int idOffset = control->calibration.markerTemplates2D.size();
	if (markerID < idOffset)
	{ // Dealing with calibration marker
		control->calibration.markerTemplate2D = &control->calibration.markerTemplates2D[markerID];
		wxLogMessage("Select calibration marker %s (%d) with %d points!", control->calibration.markerTemplate2D->label, control->calibration.markerTemplate2D->id, (int)control->calibration.markerTemplate2D->points.size());
	}
	else
	{ // Dealing with tracking marker
		control->testing.markerTemplate3D = &wxGetApp().m_state.config.testing.trackingMarkers[markerID-idOffset];
		control->tracking.trackID = control->testing.markerTemplate3D->id;
		wxLogMessage("Select test tracking marker %s (%d) with %d points!", control->testing.markerTemplate3D->label, control->testing.markerTemplate3D->id, (int)control->testing.markerTemplate3D->points.size());
	}
}
void ConfiguratorFrame::OnSelectPhase(wxCommandEvent &event)
{
	if (wxGetApp().m_state.mode == MODE_None) return;
	ControlState *control = &wxGetApp().m_state.control;
	DiscardPhase(control);
	buttonAccept->SetLabel(phaseSelector->GetSelection() == 2? "Next" : "Accept");
	switch(phaseSelector->GetSelection())
	{
		case 0: // Idle
			DiscardPhase(control);
			break;
		case 1: // Instrinsic Calibration
			EnterPhase(control, PHASE_Calibration_Intrinsic);
			break;
		case 2: // Extrinsic Calibration
			EnterPhase(control, PHASE_Calibration_Extrinsic);
			break;
		case 3: // Marker Calibration
			EnterPhase(control, PHASE_Calibration_Marker);
			break;
		case 4: // Tracking
			EnterPhase(control, PHASE_Tracking);
			break;
	}
}
void ConfiguratorFrame::OnAcceptPhase(wxCommandEvent &event)
{
	ControlState *control = &wxGetApp().m_state.control;
	FinalizePhase(control);
	if (control->lastPhase == PHASE_Calibration_Extrinsic)
	{
		EnterPhase(control, PHASE_Calibration_Room);
		phaseSelector->SetSelection(2);
	}
	else
		phaseSelector->SetSelection(0);
	buttonAccept->SetLabel("Accept");
}
void ConfiguratorFrame::OnDiscardPhase(wxCommandEvent &event)
{
	ControlState *control = &wxGetApp().m_state.control;
	DiscardPhase(control);
	phaseSelector->SetSelection(0);

}
void ConfiguratorFrame::OnChangeSetup(wxSpinEvent &event)
{
	wxGetApp().m_state.config.mode.cameraResolutionX = widthField->GetValue();
	wxGetApp().m_state.config.mode.cameraResolutionY = heightField->GetValue();
	wxGetApp().m_state.config.mode.cameraFramerate = fpsField->GetValue();
	wxGetApp().m_state.config.mode.cameraShutterSpeed = ssField->GetValue();
	
	if (wxGetApp().m_state.mode == MODE_Device)
	{
		uint16_t setupData[4];
		setupData[0] = wxGetApp().m_state.config.mode.cameraResolutionX;
		setupData[1] = wxGetApp().m_state.config.mode.cameraResolutionY;
		setupData[2] = wxGetApp().m_state.config.mode.cameraFramerate;
		setupData[3] = wxGetApp().m_state.config.mode.cameraShutterSpeed;
		comm_submit_control_data(&wxGetApp().m_state.comm, 2, 0, 0, setupData, sizeof(setupData)); // request 2
	}
}
void ConfiguratorFrame::OnKeyDown(wxKeyEvent &event)
{
	HandleInput(event, NULL);
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

// ----------------------------------------------------------------------------
// CameraFrame
// ----------------------------------------------------------------------------

CameraFrame::CameraFrame(std::string name = "MakersVR Camera View", float aspect = 3.0f/4)
	: wxFrame(NULL, wxID_ANY, name)
{
	SetClientSize(800, (int)(800 * aspect));
	Bind(wxEVT_CLOSE_WINDOW, &CameraFrame::OnClose, this);
	m_canvas = new wxGLCanvas(this, wxID_ANY, NULL, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE);
	m_canvas->Bind(wxEVT_KEY_DOWN, &CameraFrame::OnKeyDown, this);
	Show();
}
void CameraFrame::OnClose(wxCloseEvent& event)
{
	wxGetApp().OnCloseCameraFrame(this);
	Destroy();
}
void CameraFrame::OnKeyDown(wxKeyEvent &event)
{
	HandleInput(event, wxGetApp().GetCameraState(this));
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

	ControlState *control = &wxGetApp().m_state.control;
	CameraState *cam = wxGetApp().GetCameraState(this);
	if (cam == NULL)
	{
		wxLogError("No Camera for CameraFrame!");
		return;
	}
	if (control->phase == PHASE_Calibration_Intrinsic)
	{
		visualizeDistortion(cam->camera, control->testing.isTesting? cam->testing.camera : cam->camera);
		visualizePoints2D(cam->camera, cam->points2D);
		visualizeMarkers(cam->camera, cam->intrinsic.markers);
	}
	else if (control->phase == PHASE_Calibration_Extrinsic || control->phase == PHASE_Calibration_Room)
	{
//		visualizeDistortion(cam->camera, cam->testing.camera);
		visualizePoints2D(cam->camera, cam->undistorted2D);
		visualizePoses(cam->camera, cam->extrinsic.poses, true);
		visualizeMarkers(cam->camera, cam->extrinsic.markers);
	}
	else if (control->phase == PHASE_Calibration_Marker)
	{
		visualizePoints2D(cam->camera, cam->undistorted2D, { 1, 1, 1 });
		visualizeRays(cam->camera, control->testing.rays3D);
		visualizeTriangulation(cam->camera, control->tracking.points3D, control->tracking.nonconflictedCount);
		visualizePoints3D(cam->camera, control->markerCalib.markerPoints, { 0, 1, 0 }, 4, 0);
	}
	else if (control->phase == PHASE_Tracking)
	{
		visualizePoints2D(cam->camera, cam->undistorted2D, { 1, 1, 1 });
		visualizeRays(cam->camera, control->testing.rays3D);
		visualizePoses(cam->camera, control->tracking.poses3D, false);
		visualizeTriangulation(cam->camera, control->tracking.points3D, control->tracking.nonconflictedCount);
//		visualizePoints3D(cam->camera, state->testing.triangulatedPoints3D, {0,1,0}, 2.0f);
	}
	else if (control->phase == PHASE_Calibration_Marker)
	{
		visualizePoints2D(cam->camera, cam->undistorted2D, { 1, 1, 1 });
		visualizeRays(cam->camera, control->testing.rays3D);
//		visualizePoses(cam->camera, state->tracking.poses3D, false);
		visualizeTriangulation(cam->camera, control->tracking.points3D, control->tracking.nonconflictedCount);
//		visualizePoints3D(cam->camera, state->testing.triangulatedPoints3D, {0,1,0}, 2.0f);
	}
	else if (control->phase == PHASE_Idle)
		visualizePoints2D(cam->camera, cam->points2D);

	m_canvas->SwapBuffers();
}


// ----------------------------------------------------------------------------
// Keyboard input
// ----------------------------------------------------------------------------

static void HandleInput(wxKeyEvent &event, CameraState *cam)
{
	ConfiguratorState &state = wxGetApp().m_state;
	ControlState *control = &state.control;
	
	switch (event.GetKeyCode())
	{
	// Comm control
	case 'c': case 'C': // Check Debug
		if (state.mode == MODE_Device)
			comm_submit_control_request(&state.comm, 1, 0, 0); // Request 1
		break;
	case 'x': case 'X': // Iterate connected MarkerDetectors
		if (state.mode == MODE_Device)
			comm_submit_control_request(&state.comm, 2, 0, 0); // Request 2
		break;
	// State control
	case 'b': case 'B': // Finalize phase
		FinalizePhase(control);
		break;
	case 'n': case 'N': // Next phase
		NextPhase(control);
		break;
	default:
		if (cam)
		{
			const float dA = 5, dX = 10;
			switch (event.GetKeyCode())
			{
			// Following is for testing only:
			case '1':
				state.focusOnCamera = 0;
				break;
			case '2':
				state.focusOnCamera = 1;
				break;
			case '3':
				state.focusOnCamera = 2;
				break;
			// Target Marker Rotation
			case 'q': case 'Q':
				state.RGT.z() += dA/180.0f*PI;
				break;
			case 'e': case 'E':
				state.RGT.z() -= dA/180.0f*PI;
				break;
			case 'w': case 'W':
				state.RGT.x() -= dA/180.0f*PI;
				break;
			case 's': case 'S':
				state.RGT.x() += dA/180.0f*PI;
				break;
			case 'a': case 'A':
				state.RGT.y() -= dA/180.0f*PI;
				break;
			case 'd': case 'D':
				state.RGT.y() += dA/180.0f*PI;
				break;
			// Target Marker Position
			case WXK_UP:
				state.TGT.y() += dX;
				break;
			case WXK_DOWN:
				state.TGT.y() -= dX;
				break;
			case WXK_LEFT:
				state.TGT.x() -= dX;
				break;
			case WXK_RIGHT:
				state.TGT.x() += dX;
				break;
			case WXK_PAGEUP:
				state.TGT.z() += dX;
				break;
			case WXK_PAGEDOWN:
				state.TGT.z() -= dX;
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
		}
		else
		{
			event.Skip();
			return;
		}
	}
	state.updateTestThread = true;
}