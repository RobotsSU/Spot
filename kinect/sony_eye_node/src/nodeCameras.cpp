#include "sony_eye_node/nodeCameras.h"

nodeCameras::nodeCameras(ros::NodeHandle *NodeROS)
{
   m_pNode = NodeROS;
   image_transport::ImageTransport myImageTransporter(*NodeROS);
   image_transport::ImageTransport IT(*m_pNode);
   m_ImagePublisherUpward = myImageTransporter.advertise("sony_eye/right/image_raw", 100);
   m_ImagePublisherForward = myImageTransporter.advertise("sony_eye/left/image_raw", 100);
   m_CameraServer = m_pNode->advertiseService("cameras_service", &nodeCameras::CamerasServerCallback, this);
   
   left_camerainfo = m_pNode->advertise<sensor_msgs::CameraInfo>("sony_eye/left/camera_info", 100);
   right_camerainfo = m_pNode->advertise<sensor_msgs::CameraInfo>("sony_eye/right/camera_info", 100);

   m_AllDone = false;

   m_pCameras = new Cameras;
   if (m_pCameras->InitializeCameras()) std::cout << "Cameras initialized" << std::endl;
   else std::cout << "Cameras not initialized" << std::endl;
   m_image = NULL;
   m_image1 = NULL;
   m_RotatedImage = NULL;
}

nodeCameras::~nodeCameras()
{
   m_pNode->shutdown(); // exit cleanly from the master
   delete m_pCameras;
}

bool nodeCameras::CamerasServerCallback(sony_eye_node::CameraServerMessage::Request &req,
         sony_eye_node::CameraServerMessage::Response &res)
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
        //EyeIndex = req.EyeIndex;
        //m_image = cvLoadImage("/home/dbarry/Documents/ROS_projects/CameraNode/src/Hokuyo.jpg");
        m_image = m_pCameras->FrameGrab(0);
	    m_image1 = m_pCameras->FrameGrab(1);

        if (m_image && m_image1)
        {
            publishImage(0);
        }
        else
        {
            std::cout << "Failed to grab image 1" << std::endl;
            return false;
        }
        return true;
    }
    return false;   // comand not recognized
  }

void nodeCameras::publishImage(int CameraIndex)
{
    ros::Time time = ros::Time::now();  // for VSLAM we need identical time stamps

   sensor_msgs::CvBridge m_bridge;

	sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(m_image, "bgr8");
    msg->header.stamp = time;
	m_ImagePublisherForward.publish(msg); //(m_bridge.cvToImgMsg(m_image, "bgr8"));
	cvShowImage("Forward Eye", m_image);
	std::cout << "Forward image published" << std::endl;

	//  cv::WImageBuffer3_b image(m_image);
	sensor_msgs::ImagePtr msg1 = sensor_msgs::CvBridge::cvToImgMsg(m_image1, "bgr8");
    msg1->header.stamp = time;
	//m_ImagePublisherUpward.publish(m_bridge.cvToImgMsg(image, "bgr8"));
	m_ImagePublisherUpward.publish(msg1);
	cvShowImage("Upward Eye", m_image1);
	
	publishCameraInfo(time);
}

void nodeCameras::publishCameraInfo(ros::Time time)
{
	sensor_msgs::CameraInfo right_info, left_info;
	
/*	double left_D[] = {-0.090162190912025889, 0.1841703609866627, -0.0011008661563242776, -0.0024612538361511132, 0.0};
	double left_K[] = {529.70146652911649, 0.0, 305.51026468515556, 0.0, 529.49044495037401, 230.42519117958901, 0.0, 0.0, 1.0};
	double left_R[] = {0.99270623491965604, 0.065169971322569645, -0.10142586449942725, -0.062750030853502781, 0.99766741914564561, 0.026872930676634092, 0.10294058859130918, -0.020312449706576103, 0.99448008507329733};
	double left_P[] =  {651.43437521618955, 0.0, 372.01054954528809, 0.0, 0.0, 651.43437521618955, 225.1552562713623, 0.0, 0.0, 0.0, 1.0, 0.0}; */
	
	double left_D[] =  {-0.11662883643649664, 0.18069244585759339, -0.0023395930705832888, -0.002027528754243555, 0.0};
	double left_K[] =  {550.65893420901762, 0.0, 317.28465948189023, 0.0, 551.0071412248501, 222.33718145489667, 0.0, 0.0, 1.0};
	double left_R[] =  {0.99475099171826475, 0.064625954712720035, -0.079334421615074563, -0.066485453342431611, 0.99756590340466833, -0.021022674860235027, 0.077782703536070918, 0.026186911652527262, 0.9966263576128831};
	double left_P[] =  {651.43437521618955, 0.0, 372.01054954528809, 0.0, 0.0, 651.43437521618955, 225.1552562713623, 0.0, 0.0, 0.0, 1.0, 0.0};
	left_info.header.stamp = time;
	left_info.height = 480;
	left_info.width = 640;
	
	double right_D[] =  {-0.11662883643649664, 0.18069244585759339, -0.0023395930705832888, -0.002027528754243555, 0.0};
	double right_K[] =  {550.65893420901762, 0.0, 317.28465948189023, 0.0, 551.0071412248501, 222.33718145489667, 0.0, 0.0, 1.0};
	double right_R[] =  {0.99475099171826475, 0.064625954712720035, -0.079334421615074563, -0.066485453342431611, 0.99756590340466833, -0.021022674860235027, 0.077782703536070918, 0.026186911652527262, 0.9966263576128831};
	double right_P[] =  {651.43437521618955, 0.0, 372.01054954528809, -77.21302205499471, 0.0, 651.43437521618955, 225.1552562713623, 0.0, 0.0, 0.0, 1.0, 0.0};
	for (int i=0; i < 5; i++) right_info.D[i] = right_D[i];
	for (int i=0; i < 9; i++) right_info.K[i] =right_K[i];
	for (int i=0; i < 9; i++) right_info.R[i] = right_R[i];
	for (int i=0; i < 12; i++) right_info.P[i] = right_P[i];
	
	for (int i=0; i < 5; i++) left_info.D[i] = left_D[i];
	for (int i=0; i < 9; i++) left_info.K[i] =left_K[i];
	for (int i=0; i < 9; i++) left_info.R[i] = left_R[i];
	for (int i=0; i < 12; i++) left_info.P[i] = left_P[i];
	
	right_info.header.stamp = time;
	right_info.height = 480;
	right_info.width = 640;
	
	left_camerainfo.publish(left_info);
	right_camerainfo.publish(right_info);
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
