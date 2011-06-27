#include <vector>
#include "tennis_ball_detection/Circle.hpp"
#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#define DIST2_FOR_CLOSE 20

namespace tennisBallTests {
  
  bool isGreenOriginal(const std::vector<const void *> &args);
  bool isGreenKinect(const std::vector<const void *> &args);
  bool isGreenUSB(const std::vector<const void *> &args);
  bool pixelsAreClose(const CvPoint &p1, const CvPoint &p2);
};
