/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MAIN_H
#define MAIN_H

#include <vector>
#include <thread>

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

#include "comm.h"
#include "poseinference.hpp"


// the rendering context used by all GL canvases
class GLContext : public wxGLContext
{
public:
    GLContext(wxGLCanvas *canvas);
private:

};

class BlobCanvas : public wxGLCanvas
{
public:
	std::vector<Point> m_blobs;
	std::vector<Marker> m_markers;
	std::vector<Point*> m_freeBlobs;
	std::vector<Pose> m_poses;

    BlobCanvas(wxWindow *parent);

private:
    void OnPaint(wxPaintEvent& event);
    void OnKeyDown(wxKeyEvent& event);
    wxDECLARE_EVENT_TABLE();
};


class BlobFrame : public wxFrame
{
public:
	BlobCanvas *m_blobCanvas;

	BlobFrame();
	void SubmitBlobs();
private:
	void OnExit(wxCommandEvent &event);
	wxDECLARE_EVENT_TABLE();
};


class ConfiguratorFrame : public wxFrame
{
public:
	std::vector<BlobFrame*> m_blobFrames;
	ConfiguratorFrame();
	std::thread *testThread;
private:
	void OnExit(wxCommandEvent &event);
	void OnAbout(wxCommandEvent &event);
	void OnConnect(wxCommandEvent &event);
	void OnTest(wxCommandEvent &event);
	wxDECLARE_EVENT_TABLE();
	wxTextCtrl *logText;
};

class ConfiguratorApp : public wxApp
{
public:
	CommState m_commState;
	ConfiguratorFrame *m_frame;

    ConfiguratorApp() { m_glContext = NULL; m_frame = NULL; }

	virtual bool OnInit();
	virtual int OnExit();

    GLContext& GetContext(wxGLCanvas *canvas);

private:
    GLContext *m_glContext;
};

#endif // MAIN_H