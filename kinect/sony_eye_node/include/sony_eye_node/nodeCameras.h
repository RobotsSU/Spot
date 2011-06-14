// node for encoders

#ifndef NODE_CAMERAS_H
#define NODE_CAMERAS_H


#include <wx/wx.h>
#include <iostream>
#include <sony_eye_node/enums.h>
#include "ros/ros.h"

#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sony_eye_node/CamerasConsole.h"
#include "sony_eye_node/Eye.h"

#ifndef OPENCV_INCLUDES
#define OPENCV_INCLUDES
#include "opencv/cv.h"
#include "opencv/highgui.h"
//#include "opencv_latest/CvBridge.h"
#include "cv_bridge/CvBridge.h"
#include "opencv/cvwimage.h"

#endif // OPENCV_INCLUDES


//#include <image_msgs/Image.h>
#include "sony_eye_node/CameraServerMessage.h"


using namespace ros;
using namespace sony_eye_node;

class nodeCameras
{
  private:
   ros::NodeHandle *m_pNode;
   ros::Publisher m_ImageForwardBus_pub, m_ImageUpwardBus_pub, left_camerainfo, right_camerainfo;
   ros::ServiceServer m_CameraServer;
   image_transport::Publisher m_ImagePublisherUpward, m_ImagePublisherForward;
   Cameras *m_pCameras;
   IplImage *m_image, *m_image1, *m_RotatedImage;         // OpenCV image

   bool m_AllDone;

   bool CamerasServerCallback(CameraServerMessage::Request &req,
         CameraServerMessage::Response &res);

//    template <typename T> void fillImageHelperCV(T& m, IplImage* frame);    // in original function this is static ************************
//    bool fromIpl(IplImage* pcvimage, image_msgs::Image& imagemsg);


  public:
   nodeCameras(ros::NodeHandle *NodeROS);
   virtual ~nodeCameras();
   void publishImage(int CameraNumber);
   void publishCameraInfo(ros::Time time);
   bool GetAllDone() { return m_AllDone; }
};


#endif //NODE_CAMERAS
