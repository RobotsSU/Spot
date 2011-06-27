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
#define MIN_RADIUS 5
#define MAX_RADIUS 50


int hue_circles_detector(const sensor_msgs::ImageConstPtr& 
			 InputImage, std::vector<Circle> &myCircles)
{
  IplImage *myImage = NULL;
  cv::Mat inputImage;
  cv::Mat hsvImage;
  cv::Mat hueImage;
  cv::Mat satImage;
  cv::Mat binaryImage;
  cv::Mat outputImage;
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
  
  cvNamedWindow("Input to hue circles detector");
  cvNamedWindow("Circles");
  cvNamedWindow("Working image");
  cvStartWindowThread();
  
  cvShowImage("Input to hue circles detector", myImage);
IplImage* CirclesImage = cvCloneImage(myImage);  // just used to display our found circles
  // create memory storage that will contain all the dynamic data
  CvMemStorage* storage = 0;
  storage = cvCreateMemStorage(0);

   // Convert IplImage to cv::Mat
    inputImage = cv::Mat (myImage).clone ();


    // output = input
    outputImage = inputImage.clone ();

    // Convert Input image from BGR to HSV
    cv::cvtColor (inputImage, hsvImage, CV_BGR2HSV);

    // Zero Matrices
    hueImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);
    satImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);
    binaryImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);

    // HSV Channel 0 -> hueImage & HSV Channel 1 -> satImage
    int from_to[] = { 0,0, 1,1};
    cv::Mat img_split[] = { hueImage, satImage};
    cv::mixChannels(&hsvImage, 3,img_split,2,from_to,2);


    // ****************************************************
    // NOTE that i is ROWS and j is COLS
    // This is not x,y, it is rows and cols
    // 0,0 is upper left;  0, max is upper right;  max, 0 is lower left; max,max is lower right
    // ****************************************************
   
    for(int i = 0; i < outputImage.rows; i++)
    {
      for(int j = 0; j < outputImage.cols; j++)
      {
        // The output pixel is white if the input pixel
        // hue is green and saturation is reasonable
	// in low light the tennis ball has a hue around 160 - 180, saturations around 30 to 40
	// in normal light it has a hue around 20 - 40, saturations around 80 - 150
	// in very high light, the tennis ball center goes full white, so saturation drops to <10
	// in general, the background doesn't seem to come up too much with a very loose constraint
	// on saturation, so it works to allow mostly any saturation value.

        if( (hueImage.at<uchar>(i,j) > 20 && hueImage.at<uchar>(i,j) < 40 && satImage.at<uchar>(i,j) > 5))
	   // || (hueImage.at<uchar>(i,j) > 160 && hueImage.at<uchar>(i,j) < 180 && satImage.at<uchar>(i,j) > 20 ))
          binaryImage.at<uchar>(i,j) = 255;
  
        else {
 	  binaryImage.at<uchar>(i,j) = 0;
          // Clear pixel blue output channel
          outputImage.at<uchar>(i,j*3+0) = 0;
          // Clear pixel green output channel
          outputImage.at<uchar>(i,j*3+1) = 0;
          // Clear pixel red output channel
          outputImage.at<uchar>(i,j*3+2) = 0;
        }
      }
    }

    cv::Size strel_size;
    strel_size.width = 3;
    strel_size.height = 3;
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,strel_size);
    cv::morphologyEx(binaryImage, binaryImage,cv::MORPH_OPEN,strel,cv::Point(-1, -1),3);

   // Convert White on Black to Black on White by inverting the image
    cv::bitwise_not(binaryImage,binaryImage);
    // Blur the image to improve detection
    cv::GaussianBlur(binaryImage, binaryImage, cv::Size(7, 7), 2, 2 );



    cv::imshow ("Input to hue circles detector", inputImage);
    // Display Binary Image
    cv::imshow ("Working image", binaryImage);
    // Display segmented image
    //cv::imshow ("Circles", outputImage);


//start drawing all the circles

     IplImage InputToHough = binaryImage;
	CvSeq* circles =  
	cvHoughCircles( &InputToHough, storage, CV_HOUGH_GRADIENT, 1, 70, 140, 15, 20, 400); 
        //cvHoughCircles( &InputToHough, storage, CV_HOUGH_GRADIENT, 2, InputToHough.height/50, MIN_RADIUS, MAX_RADIUS );
      //output all the circles detected
      int NumCircles = circles->total;
      //cout << "\n\nFound " << NumCircles << " circles" << endl;

 if (NumCircles > MAX_CIRCLES) NumCircles = MAX_CIRCLES + 1;  // so we don't print out too many
    
      
      for( int i = 0; i < NumCircles; i++ ){ 
	
	float* p = (float*)cvGetSeqElem( circles, i );

	//cout << "x = " << p[0] << ", y = " << p[1] 
	//		 << ", radius = " << p[2] << endl;

		cvCircle(CirclesImage, 
					    cvPoint(cvRound(p[0]),
						    cvRound(p[1])), 
					    cvRound(p[2]), 
					    CV_RGB(0,255,0), 2, 8, 0 );


	    Circle myLocalCircle;
		int myColor = 0, actualThreshold = 100;
		myLocalCircle.setValues(cvRound(p[0]), cvRound(p[1]), cvRound(p[2]), myColor, actualThreshold);
	        myCircles.push_back(myLocalCircle);



	
          } // ends for NumCircles

	  cvShowImage("Circles", CirclesImage);

	return NumCircles;
  }

