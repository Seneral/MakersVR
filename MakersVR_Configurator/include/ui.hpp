/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef UI_H
#define UI_H

#include "configurator.hpp"

// wxWidgets minimal includes
#include "wxbase.hpp" // Disables unused libs, and includes wx/log.h
#include "wx/app.h"
#include "wx/window.h"
#include "wx/panel.h"
#include "wx/frame.h"
#include "wx/menu.h"
#include "wx/msgdlg.h"
#include "wx/glcanvas.h"
#include "wx/timer.h"
#include "wx/dcclient.h"
#include "wx/dcmemory.h"
#include "wx/textctrl.h"
#include "wx/log.h"
#include "wx/sizer.h"
#include "wx/choice.h"
#include "wx/button.h"
#include "wx/spinctrl.h"
#include "wx/stattext.h"

#include <vector>

class CameraFrame;
class ConfiguratorFrame;
class ConfiguratorApp;

/**
 * Main configurator logic
 */
class ConfiguratorApp : public wxApp
{
public:
	// Application state
	ConfiguratorState m_state;
	// Main GUI frame and camera frames
	ConfiguratorFrame *m_frame;
	std::vector<CameraFrame*> m_cameraFrames;

    ConfiguratorApp() { m_glContext = NULL; m_frame = NULL; }
	virtual bool OnInit();
	virtual int OnExit();

    wxGLContext *GetContext(wxGLCanvas *canvas);
    CameraState *GetCameraState(CameraFrame *frame);
	void SetupUI();
	void ResetUI();
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

	wxPanel *mainPanel;
	// Setup controls
	wxSpinCtrl *widthField;
	wxSpinCtrl *heightField;
	wxSpinCtrl *fpsField;
	wxSpinCtrl *ssField;
	wxSpinCtrl *gainField;
	wxSpinCtrl *absThresholdField;
	wxSpinCtrl *edgeThresholdField;
	// Phase controls
	wxChoice *phaseSelector;
	wxButton *buttonAccept;
	// Intrinsic controls
	wxChoice *phaseFocusSelector;
	wxStaticText *phaseStateText;

private:
	void OnClose(wxCloseEvent &event);
	void OnExit(wxCommandEvent &event);
	void OnAbout(wxCommandEvent &event);
	void OnConnect(wxCommandEvent &event);
	void OnDisconnect(wxCommandEvent &event);
	void OnStartStreaming(wxCommandEvent &event);
	void OnStopStreaming(wxCommandEvent &event);
	void OnStartTesting(wxCommandEvent &event);
	void OnStopTesting(wxCommandEvent &event);
	void OnSelectMarker(wxCommandEvent &event);
	void OnSelectPhase(wxCommandEvent &event);
	void OnSelectFocus(wxCommandEvent &event);
	void OnResetFocusState(wxCommandEvent &event);
	void OnAcceptPhase(wxCommandEvent &event);
	void OnDiscardPhase(wxCommandEvent &event);
	void OnChangeSetup(wxCommandEvent &event);
	void OnKeyDown(wxKeyEvent &event);

	wxDECLARE_EVENT_TABLE();
};

/**
 * Frame with camera view from a (simulated) MarkerDetector
 */
class CameraFrame : public wxFrame
{
public:
	CameraFrame(std::string name, float aspect);
	void Repaint();
	void AssureInit();
private:
	wxGLCanvas *m_canvas;
	void OnIssueRender(wxCommandEvent &event);
	void Render();
	void OnClose(wxCloseEvent &event);
	void OnKeyDown(wxKeyEvent &event);
};

#endif // UI_H