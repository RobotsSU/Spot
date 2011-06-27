#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "tennis_ball_detection/hue_circles_detector.h"
#include "tennis_ball_detection/pixel_tests.hpp"
#include <opencv2/highgui/highgui.hpp>

//change to isGreenUSB for little USB camera
void findCircles(const sensor_msgs::ImageConstPtr &image) {
  ROS_INFO("Looking for circles");
  std::vector<Circle> green_circles;
  hue_blob_detector(image,
		    &tennisBallTests::isGreenKinect,
		    green_circles);
  //hue_circles_detector(image, green_circles);
  ROS_INFO("There are %d circles", green_circles.size());
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ball_detector_single_camera");
  ros::NodeHandle nh;

  //open the necessary windows
  cv::namedWindow("Input image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Working image", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Circles", CV_WINDOW_AUTOSIZE);

  ros::Subscriber sub = nh.subscribe
    ("/camera/rgb/image_color", 2, findCircles);

  ros::spin();
  

}
