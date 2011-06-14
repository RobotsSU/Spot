#ifndef EYE_H
#define EYE_H

//****************************************************************************#include "wx.h"
#include <sony_eye_node/enums.h>
#include <wx/wx.h>
#include <wx/string.h>

#ifndef OPENCV_INCLUDES
#define OPENCV_INCLUDES
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include "opencv/cvwimage.h"
//#include "cvcam.h"
#endif // OPENCV_INCLUDES

class Eye
{
	public:
		Eye(int CameraNum, int EyeIndex);
		virtual ~Eye();
		bool Initialize();
		wxString GetWindowName() { return m_WindowName; }
		int GetEyeNum() { return m_CameraNum; }
		void Display();
		void Format();
		void GetProperties(double *FrameWidth, double *FrameHeight);
		void SetProperties(double FrameWidth, double FrameHeight);
		wxString Description();
		void SetResolution(int Width, int Height);
		IplImage *GetLastImage() { return m_Image; }
		IplImage *QueryFrame();
		void ShowLastImage();

	private:
		wxString m_WindowName;
		CvCapture* m_pCapture;
		IplImage *m_Image;
		int m_CameraNum, m_EyeIndex;
};

#endif // EYE_H
