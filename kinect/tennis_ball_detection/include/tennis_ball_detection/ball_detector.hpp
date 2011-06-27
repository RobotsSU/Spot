#ifndef __BALL__DETECTOR__H
#define __BALL__DETECTOR__H

#include <vector>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"

#include <actionlib/server/simple_action_server.h>
#include <tennis_ball_detection/BallDetectorAction.h>
#include <tennis_ball_detection/Circle.hpp>
#include <tennis_ball_detection/hue_circles_detector.h>
#include <geometry_msgs/PoseArray.h>
#include <string>

#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/MarkerArray.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

class BallDetector {
  
  typedef 
  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
						  sensor_msgs::CameraInfo,
						  sensor_msgs::PointCloud2> 
  DepthCameraSyncPolicy;

  typedef 
  actionlib::SimpleActionServer<tennis_ball_detection::BallDetectorAction> 
  BDAS;
private:
  
  // Node handle
  ros::NodeHandle nh_;
  
  // Images and conversion
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<DepthCameraSyncPolicy> sync_;


  // Action
  BDAS as_;
  tennis_ball_detection::BallDetectorFeedback feedback_;
  tennis_ball_detection::BallDetectorResult result_;


  // Publishers
  ros::Publisher ball_pub_, marker_pub_;

  /**True = run as a normal node, searching for faces continuously, 
   False = run as an action, wait for action call to start detection. */
  bool do_continuous_; 

  std::string out_frame, display_;
  
  double min_ball_radius_, max_ball_radius_;

  tf::TransformListener listener_;

  visualization_msgs::MarkerArray markers_;

  void goalCB();
  void preemptCB();
  void imageCBAll(const sensor_msgs::Image::ConstPtr &image, 
		  const sensor_msgs::CameraInfo::ConstPtr& cinfo,
		  const sensor_msgs::PointCloud2::ConstPtr& incloud);
  int findBalls(const sensor_msgs::PointCloud2::ConstPtr &cloud_ptr,
		const std::vector<Circle> &circles,
		const sensor_msgs::CameraInfo::ConstPtr &cinfo,
		geometry_msgs::PoseArray &centers,
		std::vector<double> &radiuses,
		std::vector<Circle> &ball_circles);

public:
  BallDetector(std::string action_name);
  ~BallDetector();
};

#endif //ball_detector.hpp
