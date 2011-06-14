#include "Eye.h"

Eye::Eye(int CameraNum, int EyeIndex)  /// CameraNum is the actual device number on this system, Eye index is the camera number of the cameras we are really using
{
	m_CameraNum = CameraNum;
	m_EyeIndex = EyeIndex;
	m_Image = NULL;

	if (m_EyeIndex == FORWARD_CAMERA) cvNamedWindow("Forward Eye", CV_WINDOW_AUTOSIZE);
	else if (m_EyeIndex == UPWARD_CAMERA) cvNamedWindow("Upward Eye", CV_WINDOW_AUTOSIZE);
	cvStartWindowThread();
}

Eye::~Eye()
{
	cvReleaseCapture(&m_pCapture);
}

bool Eye::Initialize()
{
   m_pCapture = cvCaptureFromCAM(m_CameraNum);  //********************************
   if (m_pCapture < 0) return false;         // ****************** check this syntax-- how do we check if no cameras are attached?????????????
   m_Image = QueryFrame();   // get a first image
   return true;
}

void Eye::Format()
{
	//cvcamGetProperty(m_EyeNum, CVCAM_VIDEOFORMAT, NULL);
}

void Eye::GetProperties(double *FrameWidth, double *FrameHeight)
{
	*FrameWidth = cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_WIDTH);
	*FrameHeight = cvGetCaptureProperty(m_pCapture,CV_CAP_PROP_FRAME_HEIGHT);
	//cvcamGetProperty(m_EyeNum, CVCAM_CAMERAPROPS, NULL);
}


void Eye::SetProperties(double FrameWidth, double FrameHeight)
{
   cvSetCaptureProperty(m_pCapture, CV_CAP_PROP_FRAME_WIDTH, FrameWidth);
   cvSetCaptureProperty(m_pCapture, CV_CAP_PROP_FRAME_WIDTH, FrameHeight);
}


wxString Eye::Description()
{
	//char Data[256];
	wxString TempString = _T("");
	//cvcamGetProperty(m_EyeNum, CVCAM_DESCRIPTION, Data);
	//for (int i=0; i < 256; i++) TempString << Data[i];
	return TempString;
}

//void Eye::SetResolution(int Width, int Height)
//{
	//cvcamSetProperty(m_EyeNum, CVCAM_RNDWIDTH, &Width);
	//cvcamSetProperty(m_EyeNum, CVCAM_RNDHEIGHT, &Height);
//}

// note that to get images from multiple cameras simultaneously, we should write two other methods
// one that uses cvGrabFrame() and the other that uses cvRetrieveFrame()
// then we first grab the frames from all the cameras and then go
// retrieve them.  if we do both grab and retrieve on each camera in turn, then the images
// will not be very simultaneous because cvRetrieveFrame takes a long time
// note that cvQueryFrame combines cvGrabFrame and cvRetrieveFrame into one command

IplImage *Eye::QueryFrame()
{
   for (int i=0; i < 8; i++) cvGrabFrame(m_pCapture); // it takes a few images to get to the newest one
   m_Image = cvRetrieveFrame(m_pCapture);
   if (m_EyeIndex == FORWARD_CAMERA) cvShowImage("Forward Eye", m_Image);
   if (m_EyeIndex == UPWARD_CAMERA)
   {
       cvConvertImage(m_Image, m_Image, 1);  // this flips the image vertically and is needed for the Linux code, since there is no
                                                // easy way to set the camera to do that directy.
                                                // It keeps the image orientation consistent with that used in the Windows code
                                                // The orientation displayed is that robot forward is image right, robt right is image bottom
                                                // that allows us to put the longer dimension in the forward/back direction and also
                                                // to display that direction horizontally so it fits better on the screen.
       cvShowImage("Upward Eye", m_Image);
   }
   return m_Image;
}
