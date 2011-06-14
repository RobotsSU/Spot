// node for encoders

#ifndef NODE_CAMERAS_H
#define NODE_CAMERAS_H


#include <wx/wx.h>
#include <iostream>
#include "enums.h"
#include "ros/ros.h"

#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "CamerasConsole.h"
#include "Eye.h"

#ifndef OPENCV_INCLUDES
#define OPENCV_INCLUDES
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/CvBridge.h"
//#include "opencv/cvwimage.h"
#endif // OPENCV_INCLUDES


//#include <image_msgs/Image.h>
#include "CameraServerMessage.h"


using namespace ros;


class nodeCameras
{
  private:
   ros::NodeHandle *m_pNode;
   ros::Publisher m_ImageForwardBus_pub, m_ImageUpwardBus_pub;
   ros::ServiceServer m_CameraServer;
   image_transport::Publisher m_ImagePublisherUpward, m_ImagePublisherForward;
   Cameras *m_pCameras;
   IplImage *m_image, *m_RotatedImage;         // OpenCV image

   bool m_AllDone;

   bool CamerasServerCallback(CameraServer::CameraServerMessage::Request &req,
         CameraServer::CameraServerMessage::Response &res);

//    template <typename T> void fillImageHelperCV(T& m, IplImage* frame);    // in original function this is static ************************
//    bool fromIpl(IplImage* pcvimage, image_msgs::Image& imagemsg);


  public:
   nodeCameras(ros::NodeHandle *NodeROS);
   virtual ~nodeCameras();
   void publishImage(int CameraNumber);
   bool GetAllDone() { return m_AllDone; }
};


#endif //NODE_CAMERAS
