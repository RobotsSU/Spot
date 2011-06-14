#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

void fillInCameraInfo(int height, int width, 
		      sensor_msgs::CameraInfo &cinfo) {
  cinfo.height = height;
  cinfo.width = width;
  //camera is uncalibrated
  cinfo.distortion_model = "plumb_bob";
  for (int i = 0; i < 5; i++) {
    cinfo.D.push_back(0);
  }
  for (int i = 0; i < 9; i++) {
    cinfo.K[i] = 0;
    cinfo.R[i] = 0;
  }
  for (int i = 0; i < 12; i++) {
    cinfo.P[i] = 0;
  }
  cinfo.binning_x = 0;
  cinfo.binning_y = 0;
  cinfo.roi.x_offset = 0;
  cinfo.roi.y_offset = 0;
  cinfo.height = 0;
  cinfo.width = 0;
  cinfo.roi.do_rectify = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usb_camera_node");
  ros::NodeHandle nh;
  
  CvCapture *grabber = cvCaptureFromCAM(-1);
  image_transport::ImageTransport itransporter(nh);
  image_transport::Publisher impub = 
    itransporter.advertise("/camera/rgb/image_color", 100);
  ros::Publisher cinfopub = nh.advertise<sensor_msgs::CameraInfo>
    ("/camera/rgb/camera_info", 100);
  sensor_msgs::CameraInfo cinfo;
  //check the height and width
  IplImage *iplim = cvQueryFrame(grabber);
  //fill in the camera info message
  fillInCameraInfo(iplim->height, iplim->width, cinfo);

  while(nh.ok()) {
    //grab a frame and publish it
    IplImage *iplim = cvQueryFrame(grabber);
    cv_bridge::CvImage cv_im;
    cv_im.header.stamp = ros::Time::now();
    cinfo.header.stamp = cv_im.header.stamp;
    cv::Mat im(iplim);
    cv_im.image = im;
    cv_im.encoding = "bgr8";
    cinfopub.publish(cinfo);
    impub.publish(cv_im.toImageMsg());
  }
  cvReleaseCapture(&grabber);
}
