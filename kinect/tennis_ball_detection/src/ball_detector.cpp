#include "tennis_ball_detection/ball_detector.hpp"
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "tennis_ball_detection/pixel_tests.hpp"

BallDetector::BallDetector(std::string name) : 
  it_(nh_),
  sync_(3),
  as_(nh_, name+"_action", false) {
  
  ROS_INFO("Constructing ball detector");
  
  // Action stuff
  as_.registerGoalCallback(boost::bind(&BallDetector::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&BallDetector::preemptCB, this));
  
  // Parameters
  ros::NodeHandle local_nh("~");
  local_nh.param("do_continuous",do_continuous_,true);
  local_nh.param("ball_output_frame", out_frame, std::string(""));
  local_nh.param("min_ball_radius", min_ball_radius_, 0.01);
  local_nh.param("max_ball_radius", max_ball_radius_, 0.03);
  local_nh.param("display", display_, std::string("none"));
  
  ROS_INFO("display = %s", display_.c_str());

  // Subscribe to the images and camera parameters
  image_sub_.subscribe(it_,"/camera/rgb/image_color",2);
  info_sub_.subscribe(nh_,"/camera/rgb/camera_info",3);
  cloud_sub_.subscribe(nh_,"/camera/rgb/points",3);
  sync_.connectInput(image_sub_, info_sub_, cloud_sub_);
  sync_.registerCallback(boost::bind(&BallDetector::imageCBAll, 
				     this, _1, _2, _3));
  ROS_INFO("Registered callback");

  // Advertise a position measure message.
  ball_pub_ = nh_.advertise<geometry_msgs::PoseArray>
    ("ball_centers",1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>
    ("ball_markers", 10);

  if (display_ == "local") {
    //open a CV window
    cv::namedWindow("Tennis Balls Image", CV_WINDOW_AUTOSIZE);
  }
  
  //start the action server
  as_.start();
  
  //tell ROS to use a couple threads
  ros::MultiThreadedSpinner s(2);
  ros::spin(s);
  
}

BallDetector::~BallDetector() {
  if (display_ == "local") {
    cv::destroyWindow("Tennis Balls Image");
  }
}

void BallDetector::goalCB() {
  ROS_INFO("Ball detector action started.");
  as_.acceptNewGoal();
}

void BallDetector::preemptCB() {
  ROS_INFO("Ball detector action preempted.");
  as_.setPreempted();
}


void BallDetector::imageCBAll(const sensor_msgs::Image::ConstPtr &image, 
			      const sensor_msgs::CameraInfo::ConstPtr& cinfo,
			      const sensor_msgs::PointCloud2::ConstPtr& 
			      incloud) {
    ROS_INFO("in depth callback");
  // Only run the detector if in continuous mode 
  //or the detector was turned on through an action invocation.
  if (!do_continuous_ && !as_.isActive()) {
    return;
  }

  //delete the previous rviz markers
  // for (unsigned int i = 0; i < markers_.markers.size(); i++) {
  //   markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  //   marker_pub_.publish(markers_.markers[i]);
  // }
  markers_.markers.clear();

  
  std::vector<Circle> green_circles; 
  //int ncircles = hue_circles_detector(image, green_circles);
  
  int ncircles = hue_blob_detector(image, &tennisBallTests::isGreenKinect, 
				   green_circles);
  
  //fake data i know works
  //int ncircles = 2;
  //egg
  //std::vector<Circle> orange_circles; 
  //Circle circle(65, 70, 40);
  //orange_circles.push_back(circle);
  //jersey
  //Circle circle(122, 307, 37);
  //orange_circles.push_back(circle);

  if (ncircles == 0) {
    return;
  }
  if (ncircles < 0) {
    ROS_WARN("Error when detecting circles");
    return;
  }


  geometry_msgs::PoseArray centers;
  std::vector<double> radiuses;
  std::vector<Circle> ball_circles;
  //ROS_INFO("Looking for balls");
  int nballs = findBalls(incloud, green_circles, cinfo, 
			 centers, radiuses, ball_circles);
  if (nballs == 0) {
    return;
  }
  if (nballs < 0) {
    ROS_WARN("Error when finding balls");
    return;
  }
  ROS_INFO("Found %d tennis balls", nballs);

  //bring up a picture of where the circles are in the image
  if (display_ == "local") {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      //toCvCopy would make more sense here
      //but there is a bug in cv_bridge that gives
      //the wrong colors
      cv_ptr = cv_bridge::toCvShare(image, 
				    sensor_msgs::image_encodings::BGR8);
      cv::Mat cv_out = cv_ptr->image.clone();
      for (unsigned int i = 0; i < ball_circles.size(); i++) {
	cv::circle(cv_out, 
		   cvPoint(ball_circles[i].getCenterX(),
			   ball_circles[i].getCenterY()),
		   ball_circles[i].getRadius(),
		   cv::Scalar(255,0,0), 4);
      }
      cv::imshow("Tennis Balls Image", cv_out);
      cv::waitKey(3);
    } catch (cv_bridge::Exception &e) {}
  }

  
  bool do_conversion = true;
  if (!out_frame.size()) {
    do_conversion = false;
  }
  if (do_conversion) {
    //does the conversion exist?
    try {
      tf::StampedTransform unused_transform;
      listener_.lookupTransform(std::string(centers.header.frame_id),
				out_frame, ros::Time(0), 
				unused_transform);
      
    } catch(tf::TransformException &e) {
      do_conversion = false;
      ROS_INFO("Error in lookup %s", e.what());
    }
    if (!do_conversion) {
      ROS_WARN("Ball Detection: Unable to convert from %s to %s.  Returning ball centers in frame %s", centers.header.frame_id.c_str(), out_frame.c_str(), 
	       centers.header.frame_id.c_str());
    }
  }
  
  result_.centers.poses.clear();
  if (do_conversion) {
    result_.centers.header.frame_id = out_frame;
    result_.centers.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < centers.poses.size(); i++) {
      try {
	//set the headers
	geometry_msgs::PoseStamped pose_in;
	geometry_msgs::PoseStamped pose_out;
	pose_in.header.frame_id = centers.header.frame_id;
	pose_in.header.stamp = ros::Time(0);
	pose_in.pose = centers.poses[i];
	listener_.transformPose(out_frame, pose_in, pose_out);
	result_.centers.poses.push_back(pose_out.pose);
      } catch(tf::TransformException &e) {
	do_conversion = false;
	ROS_INFO("Error in transforming %s", e.what());
      }
      if (!do_conversion) {
	ROS_WARN("Ball Detection: Unable to convert from %s to %s.  Returning ball centers in frame %s", centers.header.frame_id.c_str(), out_frame.c_str(), 
		 centers.header.frame_id.c_str());
	break;
      }
    }
  }

  if (!do_conversion) {
    result_.centers = centers;
  }
  if (!do_continuous_) {
    as_.setSucceeded(result_);
  }
  ball_pub_.publish(result_.centers);
  
  //also publish an rviz sphere for each ball
  //we could just look at poses
  //but they are confusing because i don't know
  //where the point is (arrow tip or arrow start?)


  for (unsigned int i = 0; i < result_.centers.poses.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = result_.centers.header.frame_id;
    marker.header.stamp = result_.centers.header.stamp;
    marker.ns = "ball_detection";
    marker.id = markers_.markers.size();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = result_.centers.poses[i];
    marker.scale.x = 4*radiuses[i];
    marker.scale.y = 4*radiuses[i];
    marker.scale.z = 4*radiuses[i];
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5; //transparent
    marker.lifetime = ros::Duration(1.0);
    markers_.markers.push_back(marker);
    marker_pub_.publish(marker);
  }
}

int BallDetector::findBalls(const sensor_msgs::PointCloud2::ConstPtr &cloud_ptr,
			    const std::vector<Circle> &circles,
			    const sensor_msgs::CameraInfo::ConstPtr &cinfo,
			    geometry_msgs::PoseArray &centers,
			    std::vector<double> &radiuses,
			    std::vector<Circle> &ball_circles) {
  
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cinfo);
  PointCloudXYZ cloud; 

  centers.header.frame_id = cloud_ptr->header.frame_id;
  centers.header.stamp = ros::Time::now();
  pcl::fromROSMsg(*cloud_ptr, cloud);
  double scale_factor = 2.0;
  for (unsigned int i = 0; i < circles.size(); i++) {
    //compute average depth of this circle
    double depth = 0;
    double avgx = 0;
    double avgy = 0;
    int npts = 0;
    int cx = circles[i].getCenterX();
    int cy = circles[i].getCenterY();
    //only look at the center of the circle
    int r = circles[i].getRadius()/scale_factor;
    int bottom = cy - r;
    if (bottom < 0) {
      bottom = 0;
    }
    int top = cy + r + 1;
    if (top > (int)cloud.height) {
      top = cloud.height;
    }
    //ROS_INFO("bottom = %d, top = %d", bottom, top);
    for (int y = bottom; y < top; y++) {
      int xdist = (int)(sqrt(r*r - (y - cy)*(y - cy)));
      int xmin = cx - xdist;
      if (xmin < 0) {
	xmin = 0;
      }
      int xmax = cx + xdist + 1;
      if (xmax > (int)cloud.width) {
	xmax = cloud.width;
      }
      for (int x = xmin; x < xmax; x++) {
	if (!isnan(cloud(x, y).z)) {
	  avgx += cloud(x, y).x;
	  avgy += cloud(x, y).y;
	  depth += cloud(x, y).z;
	  npts++;
	}
      }
      //ROS_INFO("y = %d, xmin = %d, xmax = %d", y, xmin, xmax);
    }
    if (npts == 0) {
      //no depth data
      ROS_INFO("No depth data for ball %d", i);
      continue;
    }
    avgx /= (double)npts;
    avgy /= (double)npts;
    depth /= (double)npts;
    double actradius = depth*scale_factor*r/cam_model.fx();
    ROS_INFO("Radius of ball %d is %f", i, actradius);
    //ROS_INFO("Avgx = %f avgy = %f depth = %f, centerx = %f, centery = %f, centerz = %f", avgx, avgy, depth, cloud(cx, cy).x, cloud(cx, cy).y, cloud(cx, cy).z);
    if (actradius >= min_ball_radius_ && actradius <= max_ball_radius_) {
      geometry_msgs::Pose pose;
      pose.position.x = avgx;
      pose.position.y = avgy;
      pose.position.z = depth;
      //it's a sphere... it's orientation doesn't matter
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      centers.poses.push_back(pose);
      radiuses.push_back(actradius);
      ball_circles.push_back(circles[i]);
    }
  }
  return centers.poses.size();
}


int main(int argc, char **argv) {
  ros::init(argc,argv,"ball_detector");

  BallDetector bd(ros::this_node::getName());

  return 0;
}


