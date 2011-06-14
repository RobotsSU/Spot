#include "nodeCameras.h"

nodeCameras::nodeCameras(ros::NodeHandle *NodeROS)
{
   m_pNode = NodeROS;
   image_transport::ImageTransport myImageTransporter(*NodeROS);
   image_transport::ImageTransport IT(*m_pNode);
   m_ImagePublisherUpward = myImageTransporter.advertise("imageUpward_bus", 100);
   m_ImagePublisherForward = myImageTransporter.advertise("imageForward_bus", 100);
   m_CameraServer = m_pNode->advertiseService("cameras_service", &nodeCameras::CamerasServerCallback, this);

   m_AllDone = false;

   m_pCameras = new Cameras;
   if (m_pCameras->InitializeCameras()) std::cout << "Cameras initialized" << std::endl;
   else std::cout << "Cameras not initialized" << std::endl;
   m_image = NULL;
   m_RotatedImage = NULL;
}

nodeCameras::~nodeCameras()
{
   m_pNode->shutdown(); // exit cleanly from the master
   delete m_pCameras;
}

bool nodeCameras::CamerasServerCallback(CameraServer::CameraServerMessage::Request &req,
         CameraServer::CameraServerMessage::Response &res)
{
    int EyeIndex;
    wxString tempString = _T("Service called, message is ");
    std::string stlstring = req.Command;
    wxString Message(stlstring.c_str(), wxConvUTF8);
    tempString << Message;
    std::cout << std::string(tempString.mb_str()) << std::endl;

    if (Message.IsSameAs(_T("Quit")))
    {
        m_AllDone = true;
        return false;
    }

    if (Message.IsSameAs(_T("FrameGrab")))
    {
        EyeIndex = req.EyeIndex;
        //m_image = cvLoadImage("/home/dbarry/Documents/ROS_projects/CameraNode/src/Hokuyo.jpg");
        m_image = m_pCameras->FrameGrab(EyeIndex);

        if (m_image)
        {
          // if (EyeIndex == FORWARD_CAMERA)
          // {
               //m_RotatedImage = m_pCameras->RotateNinetyDegrees(m_image); //RotateImage(m_image, 90);  // the forward camera is rotated 90 degrees.
               //cvShowImage("Forward Eye", m_RotatedImage);
              // cvShowImage("Forward Eye", m_image);
          // }
            publishImage(EyeIndex);
            //    cvReleaseImage(&m_RotatedImage); // have to do this to prevent memory leak *****************************************************************************************
        }
        else
        {
            std::cout << "Failed to grab image" << std::endl;
            return false;
        }
        return true;
    }
    return false;   // comand not recognized
  }

void nodeCameras::publishImage(int CameraIndex)
{

   sensor_msgs::CvBridge m_bridge;

   if (CameraIndex == FORWARD_CAMERA)
   {
      sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(m_image, "bgr8");
      m_ImagePublisherForward.publish(msg); //(m_bridge.cvToImgMsg(m_image, "bgr8"));
      cvShowImage("Forward Eye", m_image);
      std::cout << "Forward image published" << std::endl;
   }
   if (CameraIndex == UPWARD_CAMERA)
   {
    //  cv::WImageBuffer3_b image(m_image);
      sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(m_image, "bgr8");
      //m_ImagePublisherUpward.publish(m_bridge.cvToImgMsg(image, "bgr8"));
      m_ImagePublisherUpward.publish(msg);
      cvShowImage("Upward Eye", m_image);
   std::cout << "Upward image published" << std::endl;
   }
}

/*
static void nodeCameras::setBgrLayout(image_msgs::Image &image, int width, int height)
  {
    image.label = "image";
    image.encoding = "bgr";
    image.depth = "uint8";
    image.uint8_data.layout.dim.resize(3);
    image.uint8_data.layout.dim[0].label = "height";
    image.uint8_data.layout.dim[0].size = height;
    image.uint8_data.layout.dim[0].stride = height * (width * 3);
    image.uint8_data.layout.dim[1].label = "width";
    image.uint8_data.layout.dim[1].size = width;
    image.uint8_data.layout.dim[1].stride = width * 3;
    image.uint8_data.layout.dim[2].label = "channel";
    image.uint8_data.layout.dim[2].size = 3;
    image.uint8_data.layout.dim[2].stride = 3;
    image.uint8_data.data.resize(height * (width * 3));
  }
*/

/*
template <typename T> void nodeCameras::fillImageHelperCV(T& m, IplImage* frame)        // in original function this is static ************************
{
    m.layout.dim.resize(3);
    m.layout.dim.resize(3);
    m.layout.dim[0].label  = "height";
    m.layout.dim[0].size   = frame->height;
    m.layout.dim[0].stride = frame->widthStep*frame->height/sizeof(m.data[0]);
    m.layout.dim[1].label  = "width";
    m.layout.dim[1].size   = frame->width;
    m.layout.dim[1].stride = frame->widthStep/sizeof(m.data[0]);
    m.layout.dim[2].label  = "channel";
    m.layout.dim[2].size   = frame->nChannels;
    m.layout.dim[2].stride = frame->nChannels*sizeof(m.data[0]);
    m.data.resize(frame->widthStep*frame->height/sizeof(m.data[0]));
    memcpy((char*)(&m.data[0]), frame->imageData, m.data.size()*sizeof(m.data[0]));
}

bool nodeCameras::fromIpl(IplImage* pcvimage, image_msgs::Image& imagemsg)
{
    //imagemsg.header; // fill header
    imagemsg.label = "mylabel";
    switch(pcvimage->nChannels) {
    case 1:
        imagemsg.encoding = "mono";
        break;
    case 3: imagemsg.encoding = "rgb"; break;
    case 4: imagemsg.encoding = "rgba"; break;
    default:
        ROS_ERROR("unknown image format\n");
        return false;
    }

    switch(pcvimage->depth) {
    case IPL_DEPTH_8U: imagemsg.depth = "uint8"; fillImageHelperCV(imagemsg.uint8_data,pcvimage); break;
    case IPL_DEPTH_8S: imagemsg.depth = "int8"; fillImageHelperCV(imagemsg.int8_data,pcvimage); break;
    case IPL_DEPTH_16U: imagemsg.depth = "uint16"; fillImageHelperCV(imagemsg.uint16_data,pcvimage); break;
    case IPL_DEPTH_16S: imagemsg.depth = "int16"; fillImageHelperCV(imagemsg.int16_data,pcvimage); break;
    case IPL_DEPTH_32S: imagemsg.depth = "int32"; fillImageHelperCV(imagemsg.int32_data,pcvimage); break;
    case IPL_DEPTH_32F: imagemsg.depth = "float32"; fillImageHelperCV(imagemsg.float32_data,pcvimage); break;
    case IPL_DEPTH_64F: imagemsg.depth = "float64"; fillImageHelperCV(imagemsg.float64_data,pcvimage); break;
    default:
        ROS_ERROR("unsupported depth %d\n", pcvimage->depth);
        return false;
    }

    return true;
}
*/
