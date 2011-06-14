#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <math.h>
#include <ros/ros.h>
#include <opencv/cv.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>

//this is too long to type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

ros::Publisher image_pub;

void getColorData(double *invdepth, double maxz, 
		  sensor_msgs::Image &outimage);
void getBWData(double *invdepth, double maxz, 
	       sensor_msgs::Image &outimage);
void cloudcb(const sensor_msgs::PointCloud2Ptr &cloudptr);
int main(int argc, char **argv);

bool local_display;

void cloudcb(const sensor_msgs::PointCloud2Ptr &cloudptr) {
  //convert the PointCloud2 into a pcl PointCloud
  //which is a nice data type that we can work with
  PointCloudXYZ cloud;
  pcl::fromROSMsg(*cloudptr, cloud);

  //The image we will output
  sensor_msgs::Image outimage;

  //get the width and height of the image
  //from the CameraInfo topic
  const sensor_msgs::CameraInfoConstPtr infoptr =
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>
    ("/camera/rgb/camera_info");

  outimage.height = infoptr->height;
  outimage.width = infoptr->width;

  //now we need an array of inverse depths
  unsigned int currindex = 0;
  double maxz = 0, invdepth[outimage.height*outimage.width];

  //note: the image is indexed (row, col)
  //while the cloud is indexed (x,y)
  //so the indexes have to be backwards
  for (unsigned int i = 0; i < outimage.height; i++) {
    for (unsigned int j = 0; j < outimage.width; j++) {
      //the depth at the point (i, j)
      //in the image
      double zval = 0;
      if (!isnan(zval)) {
	//we have good depth data
	zval = 1.0/cloud(j,i).z;
      }
      invdepth[currindex] = zval;
      if (zval > maxz) {
	maxz = zval;
      }
      currindex++;
    }
  }

  //color image or not?
  bool use_color;
  ros::NodeHandle nh;
  nh.param("kinect_use_color", use_color, false);
  if (use_color) {
    getColorData(invdepth, maxz, outimage);
  } else {
    getBWData(invdepth, maxz, outimage);
  }

  ROS_DEBUG("Publishing image");
  //publish the message
  image_pub.publish(outimage);

  if (local_display) {
    //show it in the window
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
      cv_ptr = cv_bridge::toCvCopy
	(outimage, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_WARN("Unable to convert from ROS to CV images: %s", e.what());
      return;
    }
    cv::imshow("Depth-Color Image", cv_ptr->image);
    cv::waitKey(3);
  }
}

void getColorData(double *invdepth, double maxz, 
		  sensor_msgs::Image &outimage) {
  
  //we have three color values per pixel
  outimage.step = outimage.width*3;
  //note the backwards-ness (brg rather than rgb)
  outimage.encoding = "bgr8";
  outimage.data.resize(outimage.height*outimage.width*3);
  for (unsigned int k = 0; k < outimage.height*outimage.width; k++) {
    int scaledpt = (int)(invdepth[k]*1535/maxz);

    //this could be done better with HSV but whatever
    unsigned char r = 0, g = 0, b = 0;
    if (scaledpt < 256) {
      b = scaledpt;
    } else if (scaledpt < 512) {
      b = 255;
      g = scaledpt - 256;
    } else if (scaledpt < 768) {
      b = 767 - scaledpt;
      g = 255;
    } else if (scaledpt < 1024) {
      g = 255;
      r = scaledpt - 768;
    } else if (scaledpt < 1280) {
      g = 1279 - scaledpt;
      r = 255;
    } else if (scaledpt < 1536) {
      b = scaledpt - 1280;
      r = 255;
    }
    
    //bgr encoding
    outimage.data[3*k] = b;
    outimage.data[3*k+1] = g;
    outimage.data[3*k+2] = r;
  }
}

void getBWData(double *invdepth, double maxz, 
	       sensor_msgs::Image &outimage) {
  outimage.step = outimage.width;
  outimage.encoding = "mono8";
  outimage.data.resize(outimage.height*outimage.width);
  for (unsigned int k = 0; k < outimage.height*outimage.width; k++) {
    //scale the inverse depths to between 0 and 255 and 
    //make those our pixel values
    outimage.data[k] = (int)(invdepth[k]*255/maxz);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_to_color_node");

  ros::NodeHandle n;  

  n.param("kinect_local_display", local_display, true);
  if (local_display) {
    //pop up an OpenCV window
    cv::namedWindow("Depth-Color Image", CV_WINDOW_AUTOSIZE);
  }
  //advertise the topic on which we'll publish the image
  image_pub = n.advertise<sensor_msgs::Image>("depth_color_image", 1);
  //subscribe to the kinect point cloud
  ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1, 
				    cloudcb);
  ros::spin();
  if (local_display) {
    cv::destroyWindow("Depth-Color Image");
  }
}
