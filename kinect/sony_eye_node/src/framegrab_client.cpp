#include "ros/ros.h"
#include "sony_eye_node/CameraServerMessage.h"
#include <cstdlib>

using namespace sony_eye_node;

class FrameGrabNode
{
  public:
    FrameGrabNode()
    {
      client = n.serviceClient<CameraServerMessage>("cameras_service");
      timer1 = n.createTimer(ros::Duration(0.1), &FrameGrabNode::grabberCallback, this);
      
      ros::spin();
    }
  
    void grabberCallback(const ros::TimerEvent& e)
    {
      CameraServerMessage srv;
      srv.request.Command = "FrameGrab";
      
      if (client.call(srv))
      {
        ROS_INFO("Requested a frame.");
      }
      else
      {
        ROS_ERROR("Failed to call service.");
      }
    }
    
    ros::NodeHandle n;
    ros::ServiceClient client;
    
    ros::Timer timer1;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sony_eye_framegrab_client");

  FrameGrabNode framegrabnode;

  return 0;
}

