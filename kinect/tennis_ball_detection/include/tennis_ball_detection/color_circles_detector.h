#ifndef __COLOR_CIRCLES_DETECTOR__H
#define __COLOR_CIRCLES__DETECTOR__H

#include <vector>
#include "tennis_ball_detection/Circle.hpp"

//std::vector<Circle> color_circles_detector(const sensor_msgs::ImageConstPtr& InputImage, int color);
int color_circles_detector(const sensor_msgs::ImageConstPtr& 
					   InputImage, int color, std::vector<Circle> &myCircles);

#endif
