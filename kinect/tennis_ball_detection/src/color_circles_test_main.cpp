#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <math.h>
#include <ros/ros.h>
#include <opencv/cv.h>
//#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <tennis_ball_detection/hue_circles_detector.h>
#include <tennis_ball_detection/Circle.hpp>
//#include "/home/dbarry/Documents/harriet/kinectProjects/ballDetection/tennis_ball_detection/include/tennis_ball_dectection/Circle.hpp"
//#include "/home/dbarry/Documents/harriet/kinectProjects/ballDetection/tennis_ball_detection/include/tennis_ball_dectection/color_circles_detector.h"
#include <vector>

#define MAX_CIRCLES 10
#define MIN_RADIUS 5
#define MAX_RADIUS 30

IplImage *clonedImage;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage *myImage = NULL;
  sensor_msgs::CvBridge bridge;
  try
  {
    myImage = bridge.imgMsgToCv(msg, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  clonedImage = cvCloneImage( myImage );
  //cvShowImage("Input to blob detector", clonedImage);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "blob_detector");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ROS_INFO("Starting blob_detector with node name %s", ros::this_node::getName().c_str()) ;

	std::string image_topic;
	std::string image_topic_default = "/camera/rgb/image_color";
	nh.param("image_topic", image_topic, image_topic_default);
	//nh.getParam("image_topic", image_topic);  // can use this instead of nh.param, but then there is no default if the 							// parameter is not present, so nothing would display.

	clonedImage = NULL; // so we can be sure when we have gotten an image.
	image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);
	vector<Circle> myCircles;
	int NumCircles = -2;
	
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		if (clonedImage)
		{
			sensor_msgs::ImagePtr InputImage = sensor_msgs::CvBridge::cvToImgMsg(clonedImage, "bgr8");
			NumCircles = hue_circles_detector(InputImage, myCircles);
			cvReleaseImage(&clonedImage);
			clonedImage = NULL;  // we do not want to run the same image twice	
		} 

		ros::spinOnce();
		loop_rate.sleep(); 
	} // ends while ros OK
}

