#ifndef __HUE_CIRCLES_DETECTOR__H
#define __HUE_CIRCLES__DETECTOR__H

#include <vector>
#include "tennis_ball_detection/Circle.hpp"
#include <sensor_msgs/Image.h>

int hue_circles_detector(const sensor_msgs::ImageConstPtr& InputImage,
			 std::vector<Circle> &myCircles);
int hue_blob_detector(const sensor_msgs::ImageConstPtr& InputImage,
		      bool (*pixelTestFnc) (const std::vector<const void *> &args),
		      std::vector<Circle> &myCircles);

#endif
