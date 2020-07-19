/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "main.h"

#include "util.h"

#include <chrono>
#include <atomic>
#include <stdlib.h>

#define MEASURE_RECEIVE_RATE
//#define DEBUG_BLOBS_IN
//#define DEBUG_TRANSFER_IN

#ifdef MEASURE_RECEIVE_RATE
static std::chrono::time_point<std::chrono::steady_clock> g_lastReceiveTime;
static StatValue receiveRate;
static std::atomic<long long> g_receiveCount = 0;
#endif

const int camWidth = 1640;
const int camHeight = 1232;

enum CustomEvents
{
	ID_Connect = wxID_HIGHEST + 1,
	ID_Test = wxID_HIGHEST + 2,
};

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length);
static void onIsochronousIN(uint8_t *data, int length);
static void onInterruptIN(uint8_t *data, int length);

static void assureGLInit();
static void assureGLClean();

inline void printBuffer(uint8_t *buffer, uint8_t size)
{
	wxLogMessage("0x");
	for (int i = 0; i < size; i++) wxLogMessage("%02X", buffer[i]);
}

// ----------------------------------------------------------------------------
// ConfiguratorApp
// ----------------------------------------------------------------------------

wxIMPLEMENT_APP(ConfiguratorApp);

bool ConfiguratorApp::OnInit()
{
	// Setup usb device communication
	if (!comm_init(&m_commState)) return false;
	m_commState.onControlResponse = onControlResponse;
	m_commState.onInterruptIN = onInterruptIN;
	m_commState.onIsochronousIN = onIsochronousIN;
	// Open main frame
	m_frame = new ConfiguratorFrame();
	return true;
}
GLContext &ConfiguratorApp::GetContext(wxGLCanvas *canvas)
{
	if (!m_glContext) m_glContext = new GLContext(canvas);
	m_glContext->SetCurrent(*canvas);
	return *m_glContext;
}
int ConfiguratorApp::OnExit()
{
	assureGLClean();
	delete m_glContext;
	m_frame->Close();
	delete m_frame;
	// Clean up usb device communication
	comm_disconnect(&m_commState);
	comm_exit(&m_commState);
	return 0;
}

// ----------------------------------------------------------------------------
// ConfiguratorFrame
// ----------------------------------------------------------------------------

const int testFrameRate = 10;
static bool runTestThread;
static void ThreadSendTestData() 
{
	int index = 0;
	while (runTestThread)
	{
//		if (++index >= (1000/testFrameRate)-1)
		{
			index = 0;
			
			BlobFrame *blobFrame = wxGetApp().m_frame->m_blobFrames[0];
			if (blobFrame == NULL)
			{
				wxGetApp().m_frame->m_blobFrames.clear();
				continue;
			}

			BlobCanvas *blobCanvas = blobFrame->m_blobCanvas;

			// Generate blobs
//			int blobCount = 4;

			// Update list of blobs
			/*blobCanvas->m_blobs.resize(blobCount);
			for (int i = 0; i < blobCount; i++)
			{
				Point *point = &blobCanvas->m_blobs[i];
				point->X = (rand() % 10000) / 10000.0f;
				point->Y = (rand() % 10000) / 10000.0f;
				point->S = (rand() % 100) / 10.0f;
				//col = data[pos+5];
			}*/
			
			// Create random Ground Truth transform
			static double TGT[3] { 0, 0, 40 };
			static double RGT[3] { 0, 0, 20 };
			static const float PI = 3.14159265358979323846f;

			/*TGT[2] = (rand()%10000 / 10000.0f) * 200 + 20;
			TGT[0] = (rand()%10000 / 10000.0f) * TGT[2]/2 - TGT[2]/4;
			TGT[1] = (rand()%10000 / 10000.0f) * TGT[2]/2 - TGT[2]/4;
			RGT[0] = (rand()%10000 / 10000.0f) * 100 - 50;
			RGT[1] = (rand()%10000 / 10000.0f) * 100 - 50;
			RGT[2] = (rand()%10000 / 10000.0f) * 100 - 50;*/

			static double TD[3] { 0, 0, 0 };
			static double RD[3] { 0, 0, 0 };
			TD[2] += (rand()%10000 / 10000.0f) * 0.1 - 0.05;
			TD[0] += (rand()%10000 / 10000.0f) * 0.1 - 0.05;
			TD[1] += (rand()%10000 / 10000.0f) * 0.1 - 0.05;
			RD[0] += ((rand()%10000 / 10000.0f) * 2 - 1);
			RD[1] += ((rand()%10000 / 10000.0f) * 2 - 1);
			RD[2] += ((rand()%10000 / 10000.0f) * 2 - 1);
			TD[0] *= 0.8;
			TD[1] *= 0.8;
			TD[2] *= 0.8;
			RD[0] *= 0.9;
			RD[1] *= 0.9;
			RD[2] *= 0.9;
			TGT[0] += TD[0];
			TGT[1] += TD[1];
			TGT[2] += TD[2];
			RGT[0] += RD[0];
			RGT[1] += RD[1];
			RGT[2] += RD[2];


			// Project into camera view
			projectMarker(blobCanvas->m_blobs, TGT, RGT, 1.0f);

			//wxLogMessage("GT T: (%.1f, %.1f, %.1f)  R: (%.1f, %.1f, %.1f) \n", TGT[0], TGT[1], TGT[2], RGT[0], RGT[1], RGT[2]);

			// Create CV 2D array of marker points
			/*std::vector<cv::Point2f> marker2D;
			marker2D.resize(blobCanvas->m_blobs.size());
			for (int i = 0; i < blobCanvas->m_blobs.size(); i++)
			{
				marker2D[i].x = blobCanvas->m_blobs[i].X;
				marker2D[i].y = blobCanvas->m_blobs[i].Y;
			}*/

//			wxLogMessage("Blobs: %d, M2D: %d, M3D: %d \n", (int)blobCanvas->m_blobs.size(), (int)marker2D.size(), (int)marker3DTemplate.size());

			// Create CV 2D array of marker points
			/*std::vector<cv::Point2f> marker2D {
				cv::Point2f(blobCanvas->m_blobs[0].X, blobCanvas->m_blobs[0].Y),
				cv::Point2f(blobCanvas->m_blobs[1].X, blobCanvas->m_blobs[1].Y),
				cv::Point2f(blobCanvas->m_blobs[2].X, blobCanvas->m_blobs[2].Y),
				cv::Point2f(blobCanvas->m_blobs[3].X, blobCanvas->m_blobs[3].Y)
			};*/

			//blobCanvas->m_poses.clear();
			//Pose pose;
			//inferMarkerPose(blobCanvas->m_blobs, pose);

			// use solvePnP to recreate rotation and translation
			//cv::solvePnP(marker3DTemplate, marker2D, camMat, distort, rotation, translation, false, cv::SOLVEPNP_ITERATIVE);
			//rotation = rotation * (180.0f/PI);
			//translation = translation;

			// Found marker pose using CV, check against GT (ground truth)
			/*double TCV[3] { (double)translation(0), (double)translation(1), (double)translation(2) };
			double RCV[3] { (double)rotation(0), (double)std::abs(rotation(1)), (double)rotation(2) };
			wxLogMessage("CV T: (%.1f, %.1f, %.1f)  R: (%.1f, %.1f, %.1f) \n", TCV[0], TCV[1], TCV[2], RCV[0], RCV[1], RCV[2]);
			wxLogMessage("Error T: (%.1f, %.1f, %.1f)  R: (%.1f, %.1f, %.1f) \n", TCV[0]-TGT[0], TCV[1]-TGT[1], TCV[2]-TGT[2], RCV[0]-RGT[0], RCV[1]-RGT[1], RCV[2]-RGT[2]);
		*/

			// Get list of (potential) markers
			findMarkerCandidates(blobCanvas->m_blobs, blobCanvas->m_markers, blobCanvas->m_freeBlobs);

			// Infer the pose of these markers
			inferMarkerPoses(blobCanvas->m_markers, blobCanvas->m_poses);

			if (blobCanvas->m_poses.size() > 0)
			{ // Found marker pose using CV, check against GT (ground truth)
				double *TCV = blobCanvas->m_poses[0].translation;
				double *RCV = blobCanvas->m_poses[0].rotation;
				//wxLogMessage("CV T: (%.1f, %.1f, %.1f)  R: (%.1f, %.1f, %.1f) \n", TCV[0], TCV[1], TCV[2], RCV[0], RCV[1], RCV[2]);

				//wxLogMessage("Error T: (%.1f, %.1f, %.1f)  R: (%.1f, %.1f, %.1f) \n", TCV[0]-TGT[0], TCV[1]-TGT[1], TCV[2]-TGT[2], RCV[0]-RGT[0], RCV[1]-RGT[1], RCV[2]-RGT[2]);
			}
			else
			{
				wxLogMessage("Failed to infer marker pose! \n");
			}

			blobFrame->SubmitBlobs();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds((1000/testFrameRate)-1));
	}
}

wxBEGIN_EVENT_TABLE(ConfiguratorFrame, wxFrame)
EVT_MENU(ID_Connect, ConfiguratorFrame::OnConnect)
EVT_MENU(ID_Test, ConfiguratorFrame::OnTest)
EVT_MENU(wxID_EXIT, ConfiguratorFrame::OnExit)
EVT_MENU(wxID_ABOUT, ConfiguratorFrame::OnAbout)
wxEND_EVENT_TABLE()

ConfiguratorFrame::ConfiguratorFrame()
	: wxFrame(NULL, wxID_ANY, "MakersVR Configurator", wxPoint(50, 50), wxSize(450, 340))
{
	// Setup interaction
	wxMenu *menu = new wxMenu;
	menu->Append(ID_Connect, "&Connect", "Start connecting to a device");
	menu->Append(ID_Test, "&Test", "Start testing phase.");
	menu->AppendSeparator();
	menu->Append(wxID_ABOUT);
	menu->AppendSeparator();
	menu->Append(wxID_EXIT);
	wxMenuBar *menuBar = new wxMenuBar;
	menuBar->Append(menu, "&Configurator");
	SetMenuBar(menuBar);
	CreateStatusBar();
	
	logText = new wxTextCtrl(this, -1, "MakersVR Configurator - Log\n", wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_WORDWRAP | wxTE_READONLY);
	wxLog::SetActiveTarget(new wxLogTextCtrl(logText));

	Show();
}
void ConfiguratorFrame::OnExit(wxCommandEvent &event)
{
	runTestThread = false;
	if (testThread != NULL && testThread->joinable()) testThread->join();
	for (int i = 0; i < m_blobFrames.size(); i++)
	{
		m_blobFrames[i]->Close();
		delete m_blobFrames[i];
	}
	Close(true);
}
void ConfiguratorFrame::OnAbout(wxCommandEvent &event)
{
	wxMessageBox("MakersVR Configurator used to configure the MakersVR Tracking System",
				 "About MakersVR Configurator", wxOK | wxICON_INFORMATION);
}
void ConfiguratorFrame::OnConnect(wxCommandEvent &event)
{
	// Try to connect to usb Marker Detector
	if (comm_check_device(&wxGetApp().m_commState))
	{
		wxLogMessage("Connecting to Marker Detector...\n");
		if (comm_connect(&wxGetApp().m_commState, true))
		{
			SetStatusText("Connected to Marker Detector!");
			wxLogMessage("Connected to Marker Detector!\n");
			m_blobFrames.push_back(new BlobFrame());
		}
		else
		{
			wxLogMessage("Failed to connect to Marker Detector!\n");
			SetStatusText("Failed to connect to Marker Detector!");
		}
	}
	else
	{
		wxLogMessage("No Marker Detector connected!\n");
		SetStatusText("No Marker Detector connected!");
	}
}
void ConfiguratorFrame::OnTest(wxCommandEvent &event)
{
	SetStatusText("Starting testing phase!");
	wxLogMessage("Starting testing phase!\n");
	m_blobFrames.push_back(new BlobFrame());
	runTestThread = true;
	if (testThread == NULL)
		testThread = new std::thread(ThreadSendTestData);
}

// ----------------------------------------------------------------------------
// GLContext
// ----------------------------------------------------------------------------

static void CheckGLError()
{
	GLenum errLast = GL_NO_ERROR;
	for (;;)
	{
		GLenum err = glGetError();
		if (err == GL_NO_ERROR) return;

		// normally the error is reset by the call to glGetError() but if
		// glGetError() itself returns an error, we risk looping forever here
		// so check that we get a different error than the last time
		if (err == errLast)
		{
			wxLogError("OpenGL error state couldn't be reset.\n");
			return;
		}

		errLast = err;
		wxLogError("OpenGL error %d\n", err);
	}
}
static bool GLInit = false;
static void assureGLInit()
{
	if (!GLInit)
	{
		GLInit = true;

		// GLEW Init
		GLenum err = glewInit();
		if (GLEW_OK != err)
		{
			wxLogError("GLEW Init Error %d: %s\n", err, glewGetErrorString(err));
		}
		wxLogMessage("GLEW Init Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));

		// Init pose inference
		initPoseInference(camWidth, camHeight);
	}
}
static void assureGLClean()
{
	if (GLInit)
	{
		GLInit = false;
		// Clean pose inference
		cleanPoseInference();
	}
} 

GLContext::GLContext(wxGLCanvas *canvas)
	: wxGLContext(canvas)
{
	SetCurrent(*canvas);

	// Setup if needed

	CheckGLError();
}

// ----------------------------------------------------------------------------
// BlobFrame
// ----------------------------------------------------------------------------
// Seperate window to show a camera's blobs view

wxBEGIN_EVENT_TABLE(BlobFrame, wxFrame)
EVT_MENU(wxID_EXIT, BlobFrame::OnExit)
wxEND_EVENT_TABLE()

BlobFrame::BlobFrame()
	: wxFrame(NULL, wxID_ANY, "MakersVR Camera Blobs", wxDefaultPosition, wxDefaultSize)
{
	m_blobCanvas = new BlobCanvas(this);
	SetClientSize(camWidth/2, camHeight/2);
	Show(true);
}
void BlobFrame::OnExit(wxCommandEvent &event)
{
	delete m_blobCanvas;
	Close(true);
}
void BlobFrame::SubmitBlobs()
{
	// Store blobs and signal blobCanvas to repaint
	m_blobCanvas->Refresh(true);
}

// ----------------------------------------------------------------------------
// BlobCanvas
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(BlobCanvas, wxGLCanvas)
EVT_PAINT(BlobCanvas::OnPaint)
EVT_KEY_DOWN(BlobCanvas::OnKeyDown)
wxEND_EVENT_TABLE()

BlobCanvas::BlobCanvas(wxWindow *parent)
	// With perspective OpenGL graphics, the wxFULL_REPAINT_ON_RESIZE style
	// flag should always be set, because even making the canvas smaller should
	// be followed by a paint event that updates the entire canvas with new
	// viewport settings.
	: wxGLCanvas(parent, wxID_ANY, NULL,
				 wxDefaultPosition, wxDefaultSize,
				 wxFULL_REPAINT_ON_RESIZE | wxBORDER_NONE)
{
}

void BlobCanvas::OnPaint(wxPaintEvent &WXUNUSED(event))
{
	wxPaintDC dc(this);
	GLContext &context = wxGetApp().GetContext(this);
	assureGLInit();

	const wxSize ClientSize = GetClientSize() * GetContentScaleFactor();
	glViewport(0, 0, ClientSize.x, ClientSize.y);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	visualizePoses(m_blobs, m_markers, m_poses);

	CheckGLError();
	glFlush();
	SwapBuffers();
}

void BlobCanvas::OnKeyDown(wxKeyEvent &event)
{
	switch (event.GetKeyCode())
	{
	case WXK_SPACE:

		break;
	default:
		event.Skip();
		return;
	}
}





static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length)
{
	if (request == 0x01) 
	{ // Debug Response
		if (length != 0) 
		{
			//printf("DEBUG (%d): ", length);
			//printBuffer(data, length);
			//printf("\n");
			for (int i = 0; i < length; i++) if (data[i] == 0) data[i] = '*';
			data[length] = 0;
			wxLogMessage("DEBUG (%d): %s \n", length, data);
		}
	}
	else 
	{
		wxLogMessage("Control Response %d (%d): ", request, length);
		printBuffer(data, length);
		wxLogMessage("\n");
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
			wxLogMessage("Receive Rate: %.2fms +-%.2fms - Max %.2fms \n", receiveRate.avg, receiveRate.diff, receiveRate.max);
			receiveRate.max = 0;
		}
	}
	g_lastReceiveTime = now;
#endif

#ifdef DEBUG_TRANSFER_IN
	wxLogMessage("Isochronous IN (%d): ", length);
	data[length] = 0;
	wxLogMessage("%.*s", length, (char*)data);
	printBuffer(data, length);
	wxLogMessage("\n");
#endif

	const int headerSize = 8;
	static int blobUpdate = 0;
	blobUpdate++;
	int blobCount = (length-headerSize) / 6;

	BlobFrame *blobFrame = wxGetApp().m_frame->m_blobFrames[0];
	if (blobFrame == NULL)
	{
		wxGetApp().m_frame->m_blobFrames.clear();
		return;
	}
	BlobCanvas *blobCanvas = blobFrame->m_blobCanvas;

	// Update list of blobs
	blobCanvas->m_blobs.resize(blobCount);
	for (int i = 0; i < blobCount; i++)
	{
		Point *point = &blobCanvas->m_blobs[i];
		int pos = headerSize+i*6;
		uint16_t *blobData = (uint16_t*)&data[pos];
		point->X = (float)((double)blobData[0] / 65536.0 * camWidth);
		point->Y = (float)((double)blobData[1] / 65536.0 * camHeight);
		point->S = (float)data[pos+4]/2.0f;
		//col = data[pos+5];
	}
	
	// Get list of (potential) markers
	findMarkerCandidates(blobCanvas->m_blobs, blobCanvas->m_markers, blobCanvas->m_freeBlobs);

	// Infer the pose of these markers
	inferMarkerPoses(blobCanvas->m_markers, blobCanvas->m_poses);

	blobFrame->SubmitBlobs();

#ifdef DEBUG_BLOBS_IN
	if (blobCount > 0 || blobUpdate % 100 == 0)	
	{
		wxLogMessage("Isochronous IN (%d) %.*s %d Blobs: ", length, headerSize, data, blobCount);
//		printBuffer(data+headerSize, length-headerSize);
		for (int i = 0; i < blobCount; i++)
		{
			int pos = headerSize+i*6;
			uint16_t *blobData = (uint16_t*)&data[pos];
			float posX = (double)blobData[0] / 65536.0, posY = (double)blobData[1] / 65536.0, size = (float)data[pos+4]/2, col = data[pos+5];
			wxLogMessage("(%.1f, %.1f, %.1f) ", posX * 2048, posY * 2048, size);
		}
		wxLogMessage("\n");
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
			wxLogMessage("Receive Rate: %.2fms (%.2fms +-%.2fms) - Max %.2fms \n", receiveRate.cur, receiveRate.avg, receiveRate.diff, receiveRate.max);
			receiveRate.max = 0;
		}
	}
	g_lastReceiveTime = now;
#endif
	
#ifdef DEBUG_TRANSFER_IN
	wxLogMessage("Interrupt IN (%d): ", length);
	printBuffer(data, length);
	wxLogMessage("\n");
#endif

#ifdef DEBUG_BLOBS_IN
	if (length == 3 && data[0] == 'N' && data[1] == 'C' && data[2] == 'D')
	{ // No connected detectors
		wxLogMessage("Marker Tracker reports no connected Marker Detectors!\n");
	}
	else if (length == 3 && data[0] == 'S' && data[1] == 'T' && data[2] == 'L')
	{ // No connected detectors
		wxLogMessage("Marker Tracker reports stalled Marker Detectors!\n");
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
			wxLogMessage("Interrupt IN (%d) %s %d Blobs: ", length, message, blobCount);
			printBuffer(data+4, length-4);
			wxLogMessage("\n");
		}
	}
#endif
}