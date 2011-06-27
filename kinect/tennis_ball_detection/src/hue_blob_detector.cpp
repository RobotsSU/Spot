#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tennis_ball_detection/Circle.hpp>
#include "tennis_ball_detection/hue_circles_detector.h"
#include "tennis_ball_detection/pixel_tests.hpp"
#include <map>

#define MAX_CIRCLES 15
#define MIN_CIRCLES 3
#define MIN_RADIUS 5
#define MAX_RADIUS 100
#define MAX_DIST 25

class CvPointCmp {
public:
  CvPointCmp() {}
  bool operator() (const CvPoint &p1, const CvPoint &p2) const {
    if (p1.x != p2.x) return p1.x < p2.x;
    return p1.y < p2.y;
  }
};

typedef std::map<CvPoint, int, CvPointCmp> CvPointMap;


//private function declarations
int min(int n1, int n2);
int max(int n1, int n2);
int partitionOnEuclideanDistance(const cv::Mat &image,
				 const vector<CvPoint> &acc_pixels,
				 int **labels);

		   
int min(int n1, int n2) {
  if (n1 < n2) {
    return n1;
  }
  return n2;
}

int max(int n1, int n2) {
  if (n1 > n2) {
    return n1;
  }
  return n2;
}

//linear time partitioning... or would be if c++ implemented hash maps
//is in fact nlgn because of stupid logn maps
int partitionOnEuclideanDistance(const cv::Mat &image,
				 const vector<CvPoint> &acc_pixels,
				 int **labels) {

  int next_label = 0;
  for (unsigned int i = 0; i < acc_pixels.size(); i++) {
    CvPoint pixel = acc_pixels[i];
    int curr_label = labels[pixel.x][pixel.y];
    if (curr_label < 0) {
      for (int r = max(0, pixel.x - MAX_DIST); r < min(image.rows, pixel.x + MAX_DIST); r++) {
	for (int c = max(0, pixel.y - MAX_DIST); c < min(image.cols, pixel.y + MAX_DIST); c++) {
	  if (labels[r][c] >= 0) {
	    curr_label = labels[r][c];
	    break;
	  }
	}
      }
      if (curr_label < 0) {
	curr_label = next_label;
	next_label++;
      }
    }
    //check all pixels in a square 10x10 centered at this pixel
    for (int r = max(0, pixel.x - MAX_DIST); r < min(image.rows, pixel.x + MAX_DIST); r++) {
      for (int c = max(0, pixel.y - MAX_DIST); c < min(image.cols, pixel.y + MAX_DIST); c++) {
	if (image.at<uchar>(r, c) < 255) {
	  labels[r][c] = curr_label;
	}
      }
    }
  }  
  return next_label;
}


int hue_blob_detector(const sensor_msgs::ImageConstPtr& InputImage, 
			 bool (*pixelTestFnc)
			 (const std::vector<const void *> &args),
			 std::vector<Circle> &myCircles) {
  cv::Mat myImage;
  cv::Mat inputImage;
  cv::Mat hsvImage;
  cv::Mat hueImage;
  cv::Mat satImage;
  cv::Mat binaryImage;
  cv::Mat outputImage;

  try {
    //it would be better to use toCvCopy, of course,
    //but there is a (mostly documented) bug in toCvCopy
    //that gives the wrong image colors
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(InputImage,
							     sensor_msgs::image_encodings::BGR8);
    inputImage = cv_ptr->image.clone();
    cv::imshow("Input image", cv_ptr->image);
    cv::waitKey(3);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", InputImage->encoding.c_str());
    return -1;
  }
  
  // just used to display our found circles
  cv::Mat CirclesImage = inputImage.clone();

  // create memory storage that will contain all the dynamic data
  CvMemStorage* storage = 0;
  storage = cvCreateMemStorage(0);
  
  
  // output = input
  outputImage = inputImage.clone ();
  
  // Convert Input image from BGR to HSV
  cv::cvtColor (inputImage, hsvImage, CV_BGR2HSV);
  
  // Zero Matrices
  hueImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);
  satImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);
  cv::Mat valueImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);
  binaryImage = cv::Mat::zeros(hsvImage.rows, hsvImage.cols, CV_8U);
  
  std::vector<const void *> args_to_pixel_test;
  args_to_pixel_test.resize(5);
  args_to_pixel_test[0] = (&hueImage);
  args_to_pixel_test[1] = (&satImage);
  args_to_pixel_test[2] = (&valueImage);
  
  
  // HSV Channel 0 -> hueImage & HSV Channel 1 -> satImage
  int from_to[] = { 0,0, 1,1, 2,2};
  cv::Mat img_split[] = { hueImage, satImage, valueImage};
  cv::mixChannels(&hsvImage, 3,img_split,3,from_to,3);
  
  
  // ****************************************************
  // NOTE that i is ROWS and j is COLS
  // This is not x,y, it is rows and cols
  // 0,0 is upper left;  0, max is upper right;  max, 0 is lower left; max,max is lower right
  // ****************************************************
  

  for(int i = 0; i < outputImage.rows; i++) {
    for(int j = 0; j < outputImage.cols; j++) {
//       ROS_INFO("hue = %d, sat = %d, value = %d, r = %d, g = %d, b = %d, ih = %d, is = %d, iv = %d",
// 	       hueImage.at<uchar>(i, j),
// 	       satImage.at<uchar>(i, j),
// 	       valueImage.at<uchar>(i, j),
// 	       inputImage.at<uchar>(i, j*3),
// 	       inputImage.at<uchar>(i, j*3+1),
// 	       inputImage.at<uchar>(i, j*3+2),
// 	       hsvImage.at<uchar>(i, j*3),
// 	       hsvImage.at<uchar>(i, j*3+1),
// 	       hsvImage.at<uchar>(i, j*3+2));
      // The output pixel is white if the input pixel
      // hue is green and saturation is reasonable
      // in low light the tennis ball has a hue around 160 - 180, saturations around 30 to 40
      // in normal light it has a hue around 20 - 40, saturations around 80 - 150
      // in very high light, the tennis ball center goes full white, so saturation drops to <10
      // in general, the background doesn't seem to come up too much with a very loose constraint
      // on saturation, so it works to allow mostly any saturation value.
      
      //THE FOLLOWING IF STATEMENT IS WHAT WAS HERE BEFORE I CHANGED IT TO A FNC PTR --jlb
      //if( (hueImage.at<uchar>(i,j) > 20 && hueImage.at<uchar>(i,j) < 40 && satImage.at<uchar>(i,j) > 5))
      //BUT NOT THIS PART
      // || (hueImage.at<uchar>(i,j) > 160 && hueImage.at<uchar>(i,j) < 180 && satImage.at<uchar>(i,j) > 20 ))
      args_to_pixel_test[3] = &i;
      args_to_pixel_test[4] = &j;
      if (pixelTestFnc(args_to_pixel_test)) {
	binaryImage.at<uchar>(i,j) = 255;
      } else {
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
  cv::morphologyEx(binaryImage, binaryImage,cv::MORPH_OPEN,strel,cv::Point(-1, -1),2);
  
  // Convert White on Black to Black on White by inverting the image
  cv::bitwise_not(binaryImage,binaryImage);
  // Blur the image to improve detection
  cv::GaussianBlur(binaryImage, binaryImage, cv::Size(7, 7), 2);
  //cvSmooth(&binaryImage, &binaryImage);

  // Display Binary Image
  cv::imshow ("Working image", binaryImage);
  // Display segmented image
  //cv::imshow ("Circles", outputImage);
  
  std::vector<CvPoint> green_pixels;
  for(int i = 0; i < outputImage.rows; i++) {
    for(int j = 0; j < outputImage.cols; j++) {
      if (binaryImage.at<uchar>(i, j)  < 255) {
	green_pixels.push_back(cvPoint(i, j));
      }
    }
  }
  ROS_INFO("There are %d green pixels.", green_pixels.size());
  if (!green_pixels.size()) {
    cv::imshow("Circles", CirclesImage);
    cv::waitKey(3);
    return 0;
  }

  int **labels = new int*[binaryImage.rows];
  for (unsigned int i = 0; i < binaryImage.rows; i++) {
    labels[i] = new int[binaryImage.cols];
    for (unsigned int j = 0; j < binaryImage.cols; j++) {
      labels[i][j] = -1;
    }
  }
  ROS_INFO("Calling partition");
  int nblocks = 
    partitionOnEuclideanDistance(binaryImage,
				 green_pixels,
				 labels);
    //cv::partition(green_pixels, labels, &tennisBallTests::pixelsAreClose);
  ROS_INFO("finished partition.  there are %d partitions", nblocks);
  std::vector<CvPoint> minpts, maxpts;
  myCircles.clear();
  for (int i = 0; i < nblocks; i++) {
    minpts.push_back(cvPoint(1000, 1000));
    maxpts.push_back(cvPoint(-1, -1));
  }
  for (unsigned int i = 0; i < green_pixels.size(); i++) {
    CvPoint currpt = green_pixels[i];
    int label = labels[currpt.x][currpt.y];
    if (minpts[label].x > currpt.x) {
      minpts[label].x = currpt.x;
    }
    if (minpts[label].y > currpt.y) {
      minpts[label].y = currpt.y;
    }
    if (maxpts[label].x < currpt.x) {
      maxpts[label].x = currpt.x;
    }
    if (maxpts[label].y < currpt.y) {
      maxpts[label].y = currpt.y;
    }
  }
  for (unsigned int i = 0; i < binaryImage.rows; i++) {
    delete labels[i];
  }
  delete labels;
  
  for (int i = 0; i < nblocks; i++) {
    int radius_x = maxpts[i].x - minpts[i].x;
    int radius_y = maxpts[i].y - minpts[i].y;
    double radius = (radius_x + radius_y)/4.0;
    if (radius < MIN_RADIUS || radius > MAX_RADIUS) {
      continue;
    }
    //for a perfect circle this would be 1
    //the tennis ball is actually unlikely to be detected as a perfect circle
    //we can cut out some silly stuff by bounding this both from below and above
//     double rratio = ((double)max(radius_x, radius_y))/((double)min(radius_x, radius_y));
    
//     if (rratio > 3.0) {
//       continue;
//     }

    double center_x = (maxpts[i].x + minpts[i].x)/2.0;
    double center_y = (maxpts[i].y + minpts[i].y)/2.0;
    //ROS_INFO("center: x = %f, y = %f, radius = %d", center_x, center_y, radius);
    myCircles.push_back(Circle(center_y, center_x, radius));
    cv::circle(CirclesImage, 
	       cvPoint((int)center_y, (int)center_x),
	       cvRound(radius),
	       CV_RGB(0,255,0), 2, 8, 0 );
  }

  cv::imshow("Circles", CirclesImage);
  cv::waitKey(3);
  return myCircles.size();

}

