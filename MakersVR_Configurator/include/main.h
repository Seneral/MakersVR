/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MAIN_H
#define MAIN_H

#include "util.h"
#include "comm.h"
#include "poseinference.hpp"

#include "GL/glew.h"

// wxWidgets disable unused libs
#define wxNO_NET_LIB
#define wxNO_XML_LIB
#define wxNO_EXPAT_LIB
#define wxNO_REGEX_LIB
#define wxNO_ZLIB_LIB
#define wxNO_JPEG_LIB
#define wxNO_PNG_LIB
#define wxNO_TIFF_LIB
#define wxNO_STC_LIB
#define wxNO_HTML_LIB
#define wxNO_QA_LIB
#define wxNO_XRC_LIB
#define wxNO_AUI_LIB
#define wxNO_PROPGRID_LIB
#define wxNO_RIBBON_LIB
#define wxNO_RICHTEXT_LIB
#define wxNO_MEDIA_LIB
#define wxNO_WEBVIEW_LIB
// wxWidgets minimal includes
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

enum ConfiguratorState
{
	STATE_Idle = 0,
	STATE_Connected = 1,
	STATE_Testing = 2
};

/**
 * Frame with camera view from a (simulated) MarkerDetector
 */
class CameraFrame : public wxFrame
{
public:
	CameraFrame(std::string name);
	void Repaint();
private:
	wxGLCanvas *m_canvas;
	void OnClose(wxCloseEvent& event);
	void OnPaint(wxPaintEvent& event);
	void OnKeyDown(wxKeyEvent &event);
};

/**
 * Camera State of a MarkerDetector with currently estimated transform, blobs and estimated poses
 */
typedef struct
{
	Transform camera;
	std::vector<Point> blobs;
	std::vector<Pose> poses;
	// Deprecated:
	std::vector<Marker> m_markers;
	std::vector<Point*> m_freeBlobs;
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
	std::vector<DefMarker> m_markerData;
	// MarkerDetector windows and their state
	std::vector<MarkerDetector> m_markerDetectors;
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
	void StartTesting();
	void StopTesting();
	void OnCloseCameraFrame(CameraFrame *frame);

private:
    wxGLContext *m_glContext;
};

#endif // MAIN_H