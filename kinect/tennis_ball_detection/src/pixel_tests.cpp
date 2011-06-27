#include "tennis_ball_detection/pixel_tests.hpp"

//this doesn't seem to be working any more at all
//i hope it's not that the kinects are all different
//and just that i changed openCV versions... i hope...
bool tennisBallTests::isGreenOriginal(const std::vector<const void *> &args) {
  if (args.size() != 5) {
    ROS_ERROR("isGreen: Incorrect number of arguments");
    return false;
  }
  const cv::Mat *hueImage = static_cast<const cv::Mat *>(args[0]);
  const cv::Mat *satImage = static_cast<const cv::Mat *>(args[1]);
  const cv::Mat *valueImage = static_cast<const cv::Mat *>(args[2]);
  const int *i_ptr = static_cast<const int *>(args[3]);
  const int *j_ptr = static_cast<const int *>(args[4]);
  if (!hueImage || !satImage || !i_ptr || !j_ptr) {
    //actually, static cast is unlikely to catch this
    ROS_ERROR("isGreen: Incorrect number of images");
    return false;
  }
  int i = *i_ptr;
  int j = *j_ptr;
  
  return hueImage->at<uchar>(i,j) > 20 && hueImage->at<uchar>(i,j) < 40 && 
    satImage->at<uchar>(i,j) > 5;
}

bool tennisBallTests::isGreenKinect(const std::vector<const void *> &args) {
  if (args.size() != 5) {
    ROS_ERROR("isGreen: Incorrect number of arguments");
    return false;
  }
  const cv::Mat *hueImage = static_cast<const cv::Mat *>(args[0]);
  const cv::Mat *valueImage = static_cast<const cv::Mat *>(args[1]);
  const cv::Mat *satImage = static_cast<const cv::Mat *>(args[2]);
  const int *i_ptr = static_cast<const int *>(args[3]);
  const int *j_ptr = static_cast<const int *>(args[4]);
  if (!hueImage || !satImage || !i_ptr || !j_ptr) {
    //actually, static cast is unlikely to catch this
    ROS_ERROR("isGreen: Incorrect number of images");
    return false;
  }
  int i = *i_ptr;
  int j = *j_ptr;
  
  return hueImage->at<uchar>(i,j) > 15 && hueImage->at<uchar>(i,j) < 40
    && valueImage->at<uchar>(i,j) > 200 && valueImage->at<uchar>(i, j) < 256
    && satImage->at<uchar>(i,j) > 100 && satImage->at<uchar>(i, j) < 150;//130 (80, 150)
}

//on the USB camera... god knows why... sat is in the hue image
//value is in the sat image, hue is in the value image
bool tennisBallTests::isGreenUSB(const std::vector<const void *> &args) {
  if (args.size() != 5) {
    ROS_ERROR("isGreen: Incorrect number of arguments");
    return false;
  }
  const cv::Mat *satImage = static_cast<const cv::Mat *>(args[0]);
  const cv::Mat *valueImage = static_cast<const cv::Mat *>(args[1]);
  const cv::Mat *hueImage = static_cast<const cv::Mat *>(args[2]);
  const int *i_ptr = static_cast<const int *>(args[3]);
  const int *j_ptr = static_cast<const int *>(args[4]);
  if (!hueImage || !satImage || !i_ptr || !j_ptr) {
    //actually, static cast is unlikely to catch this
    ROS_ERROR("isGreenUSB: Incorrect number of images");
    return false;
  }
  int i = *i_ptr;
  int j = *j_ptr;
  return hueImage->at<uchar>(i,j) > 150 && hueImage->at<uchar>(i,j) < 180 && 
	 satImage->at<uchar>(i,j) > 20 && satImage->at<uchar>(i, j) < 40 &&
	 valueImage->at<uchar>(i,j) > 150 && valueImage->at<uchar>(i, j) < 200;
}

bool tennisBallTests::pixelsAreClose(const CvPoint &p1,
				     const CvPoint &p2) {
  double dist2 = (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
  if (dist2 < DIST2_FOR_CLOSE) {
    return true;
  }
  return false;
}

