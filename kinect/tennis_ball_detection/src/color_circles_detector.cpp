#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <tennis_ball_detection/Circle.hpp>
#include "tennis_ball_detection/hue_circles_detector.h"
//#include "/home/dbarry/Documents/harriet/kinectProjects/ballDetection/tennis_ball_detection/include/tennis_ball_dectection/Circle.hpp"
//#include <vector>

#define MAX_CIRCLES 10
#define MIN_CIRCLES 3
#define MIN_RADIUS 2
#define MAX_RADIUS 50


int color_circles_detector(const sensor_msgs::ImageConstPtr& 
					   InputImage, int color, std::vector<Circle> &myCircles)
{
  IplImage *myImage = NULL;
  sensor_msgs::CvBridge bridge;
  try
    {
      myImage = bridge.imgMsgToCv(InputImage, "bgr8");
    }
  catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", InputImage->encoding.c_str());
      return -1;
    }
  
  int myColor = color;	// blue, green, red correspond to 0, 1, 2
  if (myColor != 0 && myColor != 1 && myColor != 2)
    {
      myColor = 2;
      cout << "Invalid color requested, using green" << endl;
    }	
  
  cvNamedWindow("Input to color circles detector");
  cvNamedWindow("Circles");
  cvNamedWindow("Working image");
  cvStartWindowThread();
  
  cvShowImage("Input to color circles detector", myImage);
  
  int NumberOfThresholdLevelsToCheck = 10;
  int UpperCannyThreshold = 50, LowerCannyThreshold = 0;  // LowerCannyThreshold = 0 forces edges merging
  // create memory storage that will contain all the dynamic data
  CvMemStorage* storage = 0;
  storage = cvCreateMemStorage(0);
  
  CvSize sz = cvSize( myImage->width & -2, myImage->height & -2 );
  IplImage* timg = cvCloneImage(myImage); // make a copy of input image (it could change while we are working here and we do not want to have that)
  IplImage* CirclesImage = cvCloneImage(timg);  // just used to display our found circles
  IplImage* gray = cvCreateImage( sz, 8, 1 );
  IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
  IplImage* tgray;
  
  // down-scale and upscale the image to filter out the noise
  cvPyrDown( timg, pyr, 7 );
  cvPyrUp( pyr, timg, 7 );
  tgray = cvCreateImage( sz, 8, 1 );
  
  // find circles in different color planes of the image (blue, green, red)
  
  // extract the color plane
  cvSetImageCOI( timg, myColor+1 ); // Value 0 means that all channels are selected, 1 means that the first channel is selected, etc.
  cvCopy( timg, tgray, 0 );
  
  // try several threshold levels
  //for( int l = 4; l < NumberOfThresholdLevelsToCheck - 2; l++ )  // l = 1, skip canny-- too many contours
  //{
   int actualThreshold = 127, NumCircles = 0;
  for (int Thresholds = 0; Thresholds < NumberOfThresholdLevelsToCheck; Thresholds++)
  {
  int l = 1; // just do one threshold for now and we will set it 
  //cvRound((l+1)*255/NumberOfThresholdLevelsToCheck);

    // also skip first and last few thresholds.  By doing this instead of changing
    // NumberOfThresholdLevelsToCheck we keep the distance between checked thresholds smmaller
 
      // hack: use Canny instead of zero threshold level.
      // Canny helps to catch squares with gradient shading

      

	if( l == 0 )
	{
	  // apply Canny. Take the thresholds from UpperCannyThreshold and LowerCannyThreshold
	  // LowerCannyThreshold = 0 forces edges merging
	  cvCanny( tgray, gray, LowerCannyThreshold, UpperCannyThreshold, 5 );
	  // dilate canny output to remove potential
	  // holes between edge segments
	  cvDilate( gray, gray, 0, 1 );
	}
      else
	{
	  // apply threshold if l!=0:
	  //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
	  
	  cvThreshold( tgray, gray, actualThreshold, 
		       255, CV_THRESH_BINARY );
	  cvShowImage("Working image",gray);
	  //TempText = "\nImage after threshold = ";
	  //TempText << l;
	  //m_pGeneralTextBox->AppendText(TempText);
	  //cvWaitKey(0);
	}
      
      // find contours and store them all as a list
      //cvFindContours( gray, storage, &contours, sizeof(CvContour),
      //CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
      
      //get circles
      
      CvSeq* circles =  
	cvHoughCircles( gray, storage, CV_HOUGH_GRADIENT, 2, gray->height/50, MIN_RADIUS, MAX_RADIUS );
      
      //output all the circles detected
      NumCircles = circles->total;
      cout << "\n\nFound " << NumCircles;
      if (myColor == 0) cout << " blue circles "; 
      else if (myColor == 1) cout << " green circles ";
      else cout << " red circles ";
      cout << " at threshold = " << actualThreshold << endl;
      
      
      //start drawing all the circles

      if (NumCircles > MAX_CIRCLES) NumCircles = MAX_CIRCLES + 1;  // so we don't print out too many
    
      
      for( int i = 0; i < NumCircles; i++ ){ 
	
	float* p = (float*)cvGetSeqElem( circles, i );

	    	    cout << "x = " << p[0] << ", y = " << p[1] 
		 << ", radius = " << p[2] << endl;

	    if (myColor == 0) cvCircle(CirclesImage, 
				       cvPoint(cvRound(p[0]),
					       cvRound(p[1])), 
				       cvRound(p[2]), CV_RGB(0,0,255), 
				       2, 8, 0 );
	    else if (myColor == 1) cvCircle(CirclesImage, 
					    cvPoint(cvRound(p[0]),
						    cvRound(p[1])), 
					    cvRound(p[2]), 
					    CV_RGB(0,255,0), 2, 8, 0 );
	    else cvCircle(CirclesImage, 
			  cvPoint(cvRound(p[0]),
				  cvRound(p[1])), cvRound(p[2]), 
			  CV_RGB(255,0,0), 2, 8, 0 );

	    Circle myLocalCircle;
	    if (NumCircles >= MIN_CIRCLES && NumCircles <= MAX_CIRCLES) // we found a good threshold, so save the values
	    {		
		myLocalCircle.setValues(cvRound(p[0]), cvRound(p[1]), cvRound(p[2]), myColor, actualThreshold);
	        myCircles.push_back(myLocalCircle);
	    }



	
          } // ends for NumCircles

	  cvShowImage("Circles", CirclesImage);
	  cout << "See image and hit any key to continue" << endl;
	  cvWaitKey(0);

	if (NumCircles > MAX_CIRCLES) 
        {
		cout << "Exceeded maximum number of circles, increasing threshold" << endl;
		actualThreshold += 10;  // too many circles, increase the threshold
	        cvCopy(myImage, CirclesImage); //reset CirclesImage for new circles
        }
	else if (NumCircles < MIN_CIRCLES)
	{
		cout << "Only a few circles found, decreasing threshold" << endl;
		actualThreshold -= 3;  // no circles, so reduce the threshold
		cvCopy(myImage, CirclesImage); //reset CirclesImage for new circles
	}
        else
        {
		cout << "This is a reasonable number of circles, saving data and returning" << endl;
		break;
	}
  } // ends thresholds
  
  // release all the temporary images
  cvReleaseImage( &gray );
  cvReleaseImage( &pyr );
  cvReleaseImage( &tgray );
  cvReleaseImage( &CirclesImage);
  cvReleaseImage( &timg );
  
  cvClearMemStorage( storage );
  cvDestroyWindow("Input to color circles detector");
  cvDestroyWindow("Working image");
  cvDestroyWindow("Circles");
  
  if (NumCircles >= MIN_CIRCLES && NumCircles <= MAX_CIRCLES) return NumCircles;
  else return 0;  // if we never met the criteria, we didn't record any circles in the vector
}

