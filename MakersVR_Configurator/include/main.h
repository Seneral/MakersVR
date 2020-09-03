/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MAIN_H
#define MAIN_H

#include "util.h"
#include "comm.h"
#include "eigenutil.hpp"
#include "testing.hpp"
#include "calibration.hpp"
#include "tracking.hpp"

#include "GL/glew.h"

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
#include <bitset>

enum ConfiguratorState
{
	STATE_Idle = 0,
	STATE_Connected,
	STATE_TestingCalibration,
	STATE_TestingTracking
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

/**
 * Camera State of a MarkerDetector with currently estimated transform, blobs and estimated poses
 */
typedef struct
{
	Camera camera; // Calibration data
	// Points2D input
	std::vector<Point> points2D;
	// Single-Camera state (Calibration)
	std::vector<Marker> m_markers;
	std::vector<Point*> m_freeBlobs;
	std::vector<Eigen::Isometry3f> poses;
	// Multi-Camera state (Tracking)
	std::vector<Ray> rays3D;
	// Ground-Truth values used in testing
	struct {
		Eigen::Isometry3f cameraGT;
		std::bitset<MAX_MARKER_POINTS> markerPtsVisible;
	} testing;
} CameraState;

/**
 * MarkerDetector with state and optional camera view frame
 */
typedef struct {
	CameraState state;
	CameraFrame *frame;
} MarkerDetector;

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
 * Main configurator logic
 */
class ConfiguratorApp : public wxApp
{
public:
	// Main GUI frame
	ConfiguratorFrame *m_frame;
	// Application and comm state
	enum ConfiguratorState m_state;
	CommState m_commState;
	// Loaded config data
	Config m_config;
	// MarkerDetector windows and their state
	std::vector<MarkerDetector> m_markerDetectors;
	// Triangulation state
	std::vector<TriangulatedPoint> points3D;
	int nonconflictedCount;
	std::vector<std::vector<int>> conflicts;
	std::vector<Eigen::Isometry3f> poses3D;
	// Testing and visualization state
	std::vector<Ray> rays3D;
	struct {
		std::vector<Eigen::Vector3f> triangulatedPoints3D;
	} testing;
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
	void StartTesting(enum ConfiguratorState testingMode);
	void StopTesting();
	void OnCloseCameraFrame(CameraFrame *frame);

private:
    wxGLContext *m_glContext;
};

#endif // MAIN_H