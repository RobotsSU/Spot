#include "ros/ros.h"
#include "people_msgs/PositionMeasurement.h"
#include "sound_play/SoundRequest.h"

ros::Publisher talker_pub_;
std::string last_id_;

void personCallback(const people_msgs::PositionMeasurementConstPtr &pos_ptr) {
  ROS_INFO("Hello, %s.  I see you over there at (%lf %lf %lf)",
	   pos_ptr->object_id.c_str(), pos_ptr->pos.x,
	   pos_ptr->pos.y, pos_ptr->pos.z);
  if (pos_ptr->object_id.compare(last_id_)) {
    sound_play::SoundRequest msg;
    msg.sound = sound_play::SoundRequest::SAY;
    msg.command = sound_play::SoundRequest::PLAY_ONCE;
    msg.arg = "Hello there";
    talker_pub_.publish(msg);
    last_id_ = pos_ptr->object_id;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "say_hello");
  ros::NodeHandle nh;

  last_id_ = "";
  ros::Subscriber person_sub = nh.subscribe
    ("/face_detector/people_tracker_measurements",1,&personCallback);
  talker_pub_ = nh.advertise<sound_play::SoundRequest>("/robotsound",1);
  ros::spin(); 
}
