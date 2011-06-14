#include "CamerasConsole.h"


//IplImage *Cameras_LastFrame[NUMBER_OF_CAMERAS];	// contains the last frame captured by each camera

Cameras::Cameras()
{
   m_NumberOfCameras = NUMBER_OF_CAMERAS;
   for (int i = 0; i < m_NumberOfCameras; i++)
	{
		//Cameras_LastFrame[i] = NULL;
		m_pEye[i] = NULL;
		if (i == UPWARD_CAMERA)
		{
			m_EyeWidth[i] = UPWARD_CAMERA_MAX_X;
			m_EyeHeight[i] = UPWARD_CAMERA_MAX_Y;
		}
		else if (i == FORWARD_CAMERA)
		{
			m_EyeWidth[i] = FORWARD_CAMERA_MAX_X;
			m_EyeHeight[i] = FORWARD_CAMERA_MAX_Y;
		}
		else
		{
			m_EyeWidth[i] = CAMERA_MAX_X;
			m_EyeHeight[i] = CAMERA_MAX_Y;
		}
	}
	//m_CamPaused = false;
	//m_CamPlaying = false;
	//m_SetResolution = false;		// when true, the program will set the cameras resolution
	//m_ShowFormatDialog = false; // when true (and m_SetResolution = false), dialog box will appear so user can set resolutions when cameras are initialized
	//m_ShowPropertyDialog = false;	// when true the cameras properties dialog box will display when cameras are initialized so user can inspect and change settings
	//m_ShowCameraDescription = false;

	// file format for the first frame from camera 0 is MyFrame0Cam0,bmp
	// the 12th frame from camera 1 is MyFrame12Cam1.bmp
	//m_filename =  _T("MyFrame");   // This is the base name, integers will be appended for each frame captured
	//m_CamSpacer = _T("Cam");	// spacer -- befoe this is the camera number
	//m_fileformat = _T(".bmp");	// can choose bmp, jpg, etc for saving images
	//m_LastCapturedFrameFilename = _T("");
	//m_FilenameIndex = _T("");
	//m_NumberOfCapturedFrames = 0;


}

Cameras::~Cameras()
{
   for (int i=0; i < m_NumberOfCameras; i++)
	{
		//if (Cameras_LastFrame[i]) cvReleaseImage(&Cameras_LastFrame[i]);
		if (m_pEye[i]) delete m_pEye[i];
	}
}



bool Cameras::InitializeCameras()
{
   //int TotalNumCameras = cvcamGetCamerasCount();	// important for this to be the first call to cvcam routines
   int TotalNumCameras = m_NumberOfCameras + FIRST_CAMERA_DEVICE_NUMBER;   // we will just go with this until we can find the new way to get the actual number
	wxString TempString, WindowName;
	TempString = _T("\nTotal number of cameras in system = ");
	TempString << 	TotalNumCameras;
			// note that the number of cameras detected may be more than the number we wish to use
			// for example, if there is a TV tuner card installed, that will increase the number of cameras count by one
			// So we set NUMBER_OF_CAMERAS to be the actual number we wish to use
	TempString << _T("\nNumber of cameras we are using = ");
	TempString << m_NumberOfCameras;
	std::cout << std::string(TempString.mb_str()) << std::endl;
	if (TotalNumCameras < m_NumberOfCameras)
	{
		std::cout << "\nNot enough cameras attached, no cameras initialized." << std::endl;
		m_NumberOfCameras = 0;
		return false;
	}

		// Generally, the "system" cameras are the lowest numbered ones
		// these are such things as TV tuner cards.  The ones we plug into external ports get the higher numbers
		// So we can assign Eyes to cameras starting with camera number = TotalNumCameras - m_NumberOfCameras.  See this in the loop below.
		// It is important to remember that the camera number is not the same as the Eye index because of the "system' cameras

	for (int i = 0; i < m_NumberOfCameras; i++)
	{
		m_pEye[i] = new Eye(i + (TotalNumCameras - m_NumberOfCameras), i);
		TempString = _T("\nEye ");
      TempString << i;
      if (!m_pEye[i]->Initialize())
		{
		    TempString << _T(" not initialized");
		    std::cout << std::string(TempString.mb_str()) << std::endl;
		    return false;
		}
		else
		{
		   TempString << _T(" initialized");
		   TempString << _T("\nFrame Width, Height = ");
		   double EyeWidth, EyeHeight;
		   m_pEye[i]->GetProperties(&EyeWidth, &EyeHeight);
		   TempString << (int) EyeWidth << _T(", ") << (int) EyeHeight;
		   std::cout << std::string(TempString.mb_str()) << std::endl;
		}

		//if (m_SetResolution) m_pEye[i]->SetResolution(m_EyeWidth[i], m_EyeHeight[i]);
		//if (m_ShowFormatDialog) m_pEye[i]->Format();
		//if (m_ShowPropertyDialog) m_pEye[i]->Properties();
		//if (m_ShowCameraDescription)
		//{
			//TempString = _T("\nEye ");
			//TempString << i;
			//TempString << _T(" description is ");
			//TempString << m_pEye[i]->Description();
			//std::cout << std::string(TempString.mb_str());
		//}
	}
	return true;
}


/*
void Cameras::Play()
{
	if (m_CamPaused) cvcamResume();
	else if (!m_CamPlaying) cvcamStart();
	m_CamPaused = false;
	m_CamPlaying = true;
}

void Cameras::Pause()
{
	cvcamPause();
	m_CamPaused = true;
	m_CamPlaying = false;
}

void Cameras::Stop()
{
	cvcamStop();
	m_CamPaused = false;
	m_CamPlaying = false;
}
*/
IplImage *Cameras::FrameGrab(int EyeIndex)
{
	//int i = 0;
	//wxString TempString;
	if (!m_pEye[EyeIndex]) return NULL;  // if called for camera number doesn't exist return false
	return m_pEye[EyeIndex]->QueryFrame();
}

	/*
	if (m_CamPaused)
    {
        //m_pMemo->AppendText("\nCamera resumed.");
        cvcamResume();
    }
	else if (!m_CamPlaying)
    {
        m_pMemo->AppendText(_T("\nCamera started."));
        cvcamStart();
    }
	for (int j=0; j < 3; j++)	// we skip the first two frames in case it is left over from the paused image
	{
		// we set Cameras_LastFrame[EyeIndex] = NULL to avoid getting
		// the last frame captured from the previous call (if the camera does not turn on fast enough)
		if (Cameras_LastFrame[EyeIndex])
		{
			cvReleaseImage(&Cameras_LastFrame[EyeIndex]);
			Cameras_LastFrame[EyeIndex] = NULL;
		}

		while (Cameras_LastFrame[EyeIndex] == NULL)
		{
			//m_pMemo->AppendText("\nWaiting for image");
			cvWaitKey(20);
			if (i > 200)
			{
				m_pMemo->AppendText(_T("\nTimed out while waiting for image."));
				return false;
			}
			i++;
		}
	}

	TempString = "\nFrame succesfully grabbed after ";
	TempString << i;
	TempString << " wait loops.";
	m_pMemo->AppendText(TempString);


	// now put it back in the state it was in originally
	if (m_CamPaused)
    {
        //m_pMemo->AppendText("\nCamera paused.");
        Pause();
    }
	else if (!m_CamPlaying)
    {
        m_pMemo->AppendText(_T("\nCamera stopped."));
        Stop();
    }
	m_pEye[CamNum]->ShowFrame(Cameras_LastFrame[EyeIndex]);
//	wxGetApp().GetRobotFrame()->GetRobot()->pGetCortex()->pGetMemory()->StoreLastCapturedFrame(Cameras_LastFrame[EyeIndex]);
	return true;
}


void Cameras::SaveSingleFrame(int EyeIndex)
{
	m_NumberOfCapturedFrames++;
	m_LastCapturedFrameFilename = m_filename;	// basic filename
	m_LastCapturedFrameFilename << m_NumberOfCapturedFrames;
	m_LastCapturedFrameFilename.Append(m_CamSpacer);		// spacer
	m_LastCapturedFrameFilename << EyeIndex;		// camera number
	m_LastCapturedFrameFilename.Append(m_fileformat);
//	cvSaveImage(m_LastCapturedFrameFilename.c_str(), wxGetApp().GetRobotFrame()->GetRobot()->pGetCortex()->pGetMemory()->pRecallLastCapturedFrame());
}

void Cameras::SaveOneFrameFromEachCamera()
{
	m_NumberOfCapturedFrames++;
	for (int i = 0; i < m_NumberOfCameras; i++)
	{
		m_LastCapturedFrameFilename = m_filename;	// basic filename
		m_LastCapturedFrameFilename << m_NumberOfCapturedFrames;
		m_LastCapturedFrameFilename.Append(m_CamSpacer);		// spacer
		m_LastCapturedFrameFilename  << i;		// camera number
		m_LastCapturedFrameFilename.Append(m_fileformat);
		FrameGrab(i);
//		cvSaveImage(m_LastCapturedFrameFilename.c_str(), wxGetApp().GetRobotFrame()->GetRobot()->pGetCortex()->pGetMemory()->pRecallLastCapturedFrame());
	}

}

wxString Cameras::GetLastCapturedFrameFilename(int EyeIndex)
{
	m_LastCapturedFrameFilename = m_filename;	// basic filename
	m_LastCapturedFrameFilename << m_NumberOfCapturedFrames;
	m_LastCapturedFrameFilename.Append(m_CamSpacer);		// spacer
	m_LastCapturedFrameFilename  << EyeIndex;		// camera number
	m_LastCapturedFrameFilename.Append(m_fileformat);
	return m_LastCapturedFrameFilename;
}



void Cameras::ShowFormatDialog()
{
	for (int i=0; i < m_NumberOfCameras; i++) if (m_pEye[i]) m_pEye[i]->Format();
}

void Cameras::ShowPropertiesDialog()
{
	for (int i=0; i < m_NumberOfCameras; i++) if (m_pEye[i]) m_pEye[i]->Properties();
}

*/


void Cameras::RotateImage(IplImage* input, double angle, double scale)
{

     CvMat* affine = cvCreateMat(2,3,CV_32FC1);
     IplImage *output;
     output = cvCloneImage(input);
     output->origin = input->origin;
     cvZero(output);
     cv2DRotationMatrix(cvPoint2D32f(input->width/2,input->height/2),
            angle, scale, affine);
     cvWarpAffine( input, output, affine);
     cvCopyImage(output, input);
     cvNamedWindow("Rotated Window", CV_WINDOW_AUTOSIZE);
     output->width = input->height;
     output->height = input->width;
     cvShowImage("Rotated Window", output);

     cvReleaseMat( &affine);
     //cvReleaseImage(&output);
}

IplImage *Cameras::RotateNinetyDegrees(IplImage* input)  // note that the returned image should be released by the calling function or there will be a memory leak!!
{
   CvSize FlippedSize;
   FlippedSize.width = input->height;
   FlippedSize.height = input->width;
   IplImage *output = cvCreateImage(FlippedSize, input->depth, 3);
   BwImage InputImage(input);
   BwImage OutputImage(output);
   for (int i = 0; i < input->height; i++) for (int j = 0; j < input->width; j++) for (int k=0; k < 3; k++) OutputImage[j][(i*3) + k] = InputImage[i][(j*3) + k];
   cvNamedWindow("Rotated Window", CV_WINDOW_AUTOSIZE);


   // the following lines create a small horizontal line in the rotated image ( or a seg fault in the case of cvCopyImage).
   // So apparently we cannot remap the allocated space for the input image with simply swapping height and width.
   // This means we have to return the new image
   // which means we have to remember to release it somewhere in the calling code.  Unfortunate.

   //input->width = output->width;
   //input->height = output->height;
   //cvCopyImage(output,input);
   //for (int i = 0; i < output->height; i++) for (int j = 0; j < output->width; j++) for (int k=0; k < 3; k++) InputImage[i][ (j*3) + k] = OutputImage[i][ (j*3) + k];


   cvShowImage("Rotated Window", output);
   //cvReleaseImage(&output);


   return output;
}





/*
void Cameras::RotateImage(IplImage* src, int angle)
{
	//E-mail: qchen@discover.uottawa.ca
	//DiscoverLab
	//School of Information Technology & Engineering
	//University of Ottawa

	//IplImage *src = ImageList.front();
	IplImage *dst = cvCloneImage(src);

	float m[6];
    CvMat M = cvMat(2, 3, CV_32F, m);
    int w = src->width;
    int h = src->height;

	m[0] = (float)(cos(-angle*2*CV_PI/180.)); // this seems to be too big by a factor of 2  It works well with the *2 removed
    m[1] = (float)(sin(-angle*2*CV_PI/180.));   // this seems to be too big by a factor of 2  It works well with the *2 removed
    m[3] = -m[1];
    m[4] = m[0];
    m[2] = w*0.5f;
    m[5] = h*0.5f;

    cvGetQuadrangleSubPix(src, dst, &M);
   cvNamedWindow("Rotated Window", CV_WINDOW_AUTOSIZE);
   cvShowImage("Rotated Window", dst);
   cvCopyImage(dst, src);
   cvReleaseImage(&dst);
}

*/
