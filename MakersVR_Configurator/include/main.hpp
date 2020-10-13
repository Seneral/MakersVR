/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MAIN_H
#define MAIN_H

#include "eigenutil.hpp"
#include "util.hpp"
#include "control.hpp"
#include "calibration.hpp"
#include "tracking.hpp"
#include "comm.hpp"

// wxWidgets minimal includes
#include "wxbase.hpp" // Disables unused libs, and includes wx/log.h
#include "wx/app.h"
#include "wx/window.h"
#include "wx/frame.h"
#include "wx/menu.h"
#include "wx/msgdlg.h"
#include "wx/glcanvas.h"
#include "wx/timer.h"
#include "wx/dcclient.h"
#include "wx/dcmemory.h"
#include "wx/textctrl.h"
#include "wx/log.h"

#include <vector>
#include <thread>

class CameraFrame;
class ConfiguratorFrame;
class ConfiguratorApp;

enum ConfiguratorState
{
	STATE_Idle = 0,
	STATE_Connected,
	STATE_Testing
};

/**
 * Main configurator logic
 */
class ConfiguratorApp : public wxApp
{
public:
	Config m_config;
	// Application state
	enum ConfiguratorState m_state;
	CommState m_commState;
	TrackingState m_globalState;
	// Main GUI frame and camera frames
	ConfiguratorFrame *m_frame;
	std::vector<CameraFrame*> m_cameraFrames;
	// Test thread
	bool runTestThread;
	std::thread *testThread;

    ConfiguratorApp() { m_glContext = NULL; m_frame = NULL; testThread = NULL; m_commState.libusb = NULL; }
	virtual bool OnInit();
	virtual int OnExit();
    wxGLContext *GetContext(wxGLCanvas *canvas);
    CameraState *GetCameraState(CameraFrame *frame);
    bool SetupComm();
	void Connect();
	void Disconnect();
	void StartTesting(enum ControlPhase phase);
	void StopTesting();
	void OnCloseCameraFrame(CameraFrame *frame);

private:
    wxGLContext *m_glContext;
};

/**
 * Main configurator window with controls
 */
class ConfiguratorFrame : public wxFrame
{
public:
	ConfiguratorFrame();
private:
	void OnExit(wxCommandEvent &event);
	void OnAbout(wxCommandEvent &event);
	void OnConnect(wxCommandEvent &event);
	void OnDisconnect(wxCommandEvent &event);
	void OnStartTesting(wxCommandEvent &event);
	void OnStopTesting(wxCommandEvent &event);
	void OnSelectMarker(wxCommandEvent &event);
	void OnClose(wxCloseEvent& event);
	wxDECLARE_EVENT_TABLE();
};

/**
 * Frame with camera view from a (simulated) MarkerDetector
 */
class CameraFrame : public wxFrame
{
public:
	CameraFrame(std::string name);
	void Repaint();
	void AssureInit();
private:
	wxGLCanvas *m_canvas;
	void OnClose(wxCloseEvent& event);
	void OnPaint(wxPaintEvent& event);
	void OnKeyDown(wxKeyEvent &event);
	void Render();
};

#endif // MAIN_H