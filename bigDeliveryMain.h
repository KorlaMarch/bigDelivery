/***************************************************************
 * Name:      bigDeliveryMain.h
 * Purpose:   Defines Application Frame
 * Author:    Khemarat Boonyapaluk (korla.march@gmail.com)
 * Created:   2018-07-16
 * Copyright: Khemarat Boonyapaluk (korlamarch.com)
 * License:
 **************************************************************/

#ifndef BIGDELIVERYMAIN_H
#define BIGDELIVERYMAIN_H

//(*Headers(white_bot_2Frame)
#include "MyGLCanvas.h"
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/menu.h>
#include <wx/sizer.h>
#include <wx/statusbr.h>
#include <wx/textctrl.h>
#include <wx/tglbtn.h>
#include <wx/timer.h>
//*)

#include <mrpt/opengl.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPlanarLaserScan.h>	

#include <mrpt/gui.h>
#include <mrpt/gui/WxUtils.h>

#include <mrpt/nav.h>

// To avoid conflicts between Eigen & X11 headers
#ifdef Success
#	undef Success
#endif

#include "BotModel.h"

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::nav;

class bigDeliveryFrame: public wxFrame
{
    public:

        bigDeliveryFrame(wxWindow* parent,wxWindowID id = -1, BotModel* botData = 0);
        virtual ~bigDeliveryFrame();

    private:

        //(*Handlers(bigDeliveryFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnbtnStartClick(wxCommandEvent& event);
        void OnbtnStopClick(wxCommandEvent& event);
        void OnbtnManualClick(wxCommandEvent& event);
        void OnbtnAutoClick(wxCommandEvent& event);
        void OnbtnLoadMapClick(wxCommandEvent& event);
        void OnbtnSetTargetToggle(wxCommandEvent& event);
        void OntimUpdate3DTrigger(wxTimerEvent& event);
        void OnbtnResetClick(wxCommandEvent& event);
        void OnbtnSetWaypointsToggle(wxCommandEvent& event);
        void OnbtnFindPathClick(wxCommandEvent& event);
        void OnbtnResetPDFClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(bigDeliveryFrame)
        static const long ID_CUSTOM1;
        static const long ID_CUSTOM2;
        static const long ID_TEXTCTRL1;
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_BUTTON3;
        static const long ID_TOGGLEBUTTON1;
        static const long ID_BUTTON8;
        static const long ID_BUTTON4;
        static const long ID_BUTTON5;
        static const long ID_BUTTON6;
        static const long ID_TOGGLEBUTTON2;
        static const long ID_BUTTON7;
        static const long ID_MENUITEM1;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long ID_TIMER1;
        //*)

        //(*Declarations(bigDeliveryFrame)
        CMyGLCanvas* m_plot3D;
        CMyGLCanvas* m_plotLaser;
        wxButton* btnAuto;
        wxButton* btnFindPath;
        wxButton* btnLoadMap;
        wxButton* btnManual;
        wxButton* btnReset;
        wxButton* btnResetPDF;
        wxButton* btnStart;
        wxButton* btnStop;
        wxStatusBar* StatusBar1;
        wxTextCtrl* textStatus;
        wxTimer timUpdate3D;
        wxToggleButton* btnSetTarget;
        wxToggleButton* btnSetWaypoints;
        //*)

		DECLARE_EVENT_TABLE()

		/**  The state of the cursor onto the 3D view */
		enum TCursorPickState
		{
			cpsNone = 0,
			cpsPickTarget,
			cpsPickWaypoints
		};

		// Our program's data
		BotModel* botData;
		mrpt::math::TPoint2D             m_curCursorPos; // Pos of cursor on the 3D view (in world coordinates at Z=0)
		TCursorPickState                 m_cursorPickState;   // The state of the cursor onto the 3D view:
		mrpt::utils::CTicTac             m_runtime; // for animations
		mrpt::nav::TWaypointSequence     m_waypoints_clicked;

		// ========= Opengl View: Map & robot  =======
		mrpt::opengl::CSetOfObjectsPtr		gl_grid;
		mrpt::opengl::CSetOfObjectsPtr		gl_robot, gl_robot_local, gl_target;
		mrpt::opengl::CSetOfObjectsPtr		m_gl_placing_nav_target, m_gl_placing_robot, m_gl_drawing_obs;
		mrpt::opengl::CDiskPtr		        gl_robot_sensor_range;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_path;
		mrpt::opengl::CPlanarLaserScanPtr   gl_scan3D;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_ptg_prediction;
		mrpt::opengl::CSetOfObjectsPtr      gl_waypoints_clicking, gl_waypoints_status;
		mrpt::opengl::CEllipsoidPtr			gl_loc_cov;

		// Methods
		void updateMap3DView();
		void updateOpenglObjects();

		void Onplot3DMouseClick(wxMouseEvent& event);
		void Onplot3DMouseMove(wxMouseEvent& event);
};

#endif // BIGDELIVERYMAIN_H
