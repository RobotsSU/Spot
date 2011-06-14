Running:

rosmake depth_color
rosrun depth_color depth_to_color

Viewing the image:

By default, an openCV window should pop up showing the image.  You can
disable that by setting the parameter "kinect_local_display" to
false.  You can always also view the image using:

rosrun image_view image_view image:=/depth_color_image

The parameter is checked once at the beginning of the program so it
cannot be changed dynamically.

Image Color:

By default, the image is greyscale with lighter values corresponding
to closer to the kinect.   

If you set the parameter "kinect_use_color" to true, the image is in
color with purple/red corresponding to closer to the kinect, green
middle distance, and blue far away.

The parameter is checked at each callback so it can be changed
dynamically.

Source code:

The source code is in src/depth_to_color.cpp.  I commented it so it
should be pretty followable. 
