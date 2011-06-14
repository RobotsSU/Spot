#ifndef CAMERAS_H
#define CAMERAS_H

#include <wx/wx.h>
#ifndef OPENCV_INCLUDES
#define OPENCV_INCLUDES
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include "opencv/cvwimage.h"
//#include "cvcam.h"

#endif // OPENCV_INCLUDES

#include "sony_eye_node/Eye.h"
#include <sony_eye_node/enums.h>
#include "sony_eye_node/IndividualPixels.h"


class Cameras
{
	public:
		Cameras();
		virtual ~Cameras();
		bool InitializeCameras();
		IplImage *FrameGrab(int EyeIndex = 0);
      void RotateImage(IplImage* input, double angle, double scale = 1.);
      //void RotateImage(IplImage* src, int angle);
      IplImage *RotateNinetyDegrees(IplImage* input);



		//Eye *GetEye(int EyeIndex) { return m_pEye[EyeIndex]; }
		//void Play();
		//void Pause();
		//void Stop();
		//void SaveSingleFrame(int EyeIndex);
		//void SaveOneFrameFromEachCamera();
		//void ShowFormatDialog();
		//void GetPropertiesDialog();
		//bool FrameGrab(int CamNum);
		//int GetNumberOfCameras() { return m_NumberOfCameras; }
		//wxString GetLastCapturedFrameFilename(int EyeIndex);


	private:

		Eye * m_pEye[NUMBER_OF_CAMERAS];
		//wxString m_filename, m_CamSpacer, m_fileformat, m_FilenameIndex, m_LastCapturedFrameFilename;
		int m_NumberOfCapturedFrames, m_NumberOfCameras;
		int m_EyeWidth[NUMBER_OF_CAMERAS], m_EyeHeight[NUMBER_OF_CAMERAS];
		bool m_CamPaused, m_CamPlaying, m_SetResolution, m_ShowFormatDialog, m_ShowPropertyDialog, m_ShowCameraDescription;


};

#endif // CAMERAS_H
