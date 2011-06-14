/*********************************************************************
 * Faces-specific computer vision algorithms.
 *
 **********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Caroline Pantofaru */

#include "kinect_face_detection/faces.h"
#include <cfloat>
#include <math.h>

namespace people {

Faces::Faces():
  list_(NULL), 
  cam_model_(NULL) {
}

Faces::~Faces() {
  // Kill all the threads.
  threads_.interrupt_all();
  threads_.join_all();
  delete face_go_mutex_;
  face_go_mutex_ = NULL;

  for (int i=list_.size(); i>0; i--) {
    list_.pop_back();
  }
  cam_model_ = NULL;
}


/* Note: The multi-threading in this file is left over from a previous incarnation that allowed multiple 
 * cascades to be run at once. It hasn't been removed in case we want to return to that model, and since 
 * there isn't much overhead. Right now, however, only one classifier is being run per instantiation 
 * of the face_detector node.
 */


void Faces::initFaceDetection(uint num_cascades, string haar_classifier_filename, double face_size_min_m, double face_size_max_m, double max_face_z_m, double face_sep_dist_m) {
  images_ready_ = 0;

  face_size_min_m_ = face_size_min_m;
  face_size_max_m_ = face_size_max_m;
  max_face_z_m_ = max_face_z_m;
  face_sep_dist_m_ = face_sep_dist_m;

  face_go_mutex_ = new boost::mutex();
  bool cascade_ok = cascade_.load(haar_classifier_filename);
  
  if (!cascade_ok) {
    ROS_ERROR_STREAM("Cascade file " << haar_classifier_filename << " doesn't exist.");
    return;
  }
  threads_.create_thread(boost::bind(&Faces::faceDetectionThread,this,0));
}

/////

vector<Box2D3D> Faces::detectAllFaces(cv::Mat &image, double threshold, 
				      image_geometry::PinholeCameraModel 
				      *cam_model,
				      const PointCloudXYZ &cloud) {

  faces_.clear();

  // Convert the image to grayscale, if necessary.
  
  if (image.channels() == 1) {
    cv_image_gray_.create(image.size(), CV_8UC1);
    image.copyTo(cv_image_gray_);
  }
  else if (image.channels() == 3) {
    cv_image_gray_.create(image.size(), CV_8UC1);
    cv::cvtColor(image, cv_image_gray_, CV_BGR2GRAY);
  }
  else {
    std::cerr << "Unknown image type"<<std::endl;
    return faces_;
  }

  cam_model_ = cam_model;
  cloud_ = cloud;
 
  // look at the depth of the middle point of the depth cloud, which we hope corresponds to the middle point of the image
  
  //wheeee!  cloud has 307200 points! wheeeeee!
  ROS_INFO("Image size = %d X %d Number of points = %d",cam_model_->height(), cam_model_->width(), cloud.size());
  // for (int y = 0; y < cam_model_->height(); y++) {
  //   for (int x = 0; x < cam_model_->width(); x++) {
  //     cout << "(" << cloud_(x, y).x << " " << cloud_(x, y).y 
  // 	   << " " << cloud_(x, y).z << ") ";
  //   }
  //   cout << "\n";
  // }
	

  // //this should *definitely* be linear interpolation or something smarter!
  // for (unsigned int index = 0; 
  //      index < cam_model_->width()*cam_model_->height(); index++) {
  //   for (unsigned int curr_diff = 0; 
  // 	 curr_diff < cam_model_->width()*cam_model_->height(); curr_diff++) {
  //     if ((index + curr_diff) < cam_model_->width()*cam_model_->height()
  // 	  && depthMap_[index + curr_diff].z < 1e8) {
  // 	//ROS_INFO("Replacing %d with %d", 
  // 	//	 index, index+curr_diff);
  // 	depthMap_[index] = depthMap_[index + curr_diff];
  // 	break;
  //     } else if (index >= curr_diff &&
  // 		 depthMap_[index - curr_diff].z < 1e8) {
  // 	//ROS_INFO("Replacing %d with %d", index, index-curr_diff);
  // 	depthMap_[index] = depthMap_[index - curr_diff];
  // 	break;
  //     }
  //   }
  // }
  
  // Tell the face detection threads that they can run once.
  num_threads_to_wait_for_ = threads_.size();
  boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
  images_ready_++;
  fgmlock.unlock();

  face_detection_ready_cond_.notify_all();

  boost::mutex::scoped_lock fdmlock(face_done_mutex_);
  while (num_threads_to_wait_for_ > 0) {
    face_detection_done_cond_.wait(fdmlock);
  }  

  return faces_;
}

/////

void Faces::faceDetectionThread(uint i) {

  while (1) {
    boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
    boost::mutex::scoped_lock tlock(t_mutex_, boost::defer_lock);
    while (1) {
      tlock.lock();
      if (images_ready_) {
	--images_ready_;
	tlock.unlock();
	break;
      }
      tlock.unlock();
      face_detection_ready_cond_.wait(fgmlock);
    }

    // Find the faces using OpenCV's haar cascade object detector.
    cv::Point3d p3_1(0,0,max_face_z_m_);
    cv::Point3d p3_2(face_size_min_m_,0,max_face_z_m_);
    cv::Point2d p2_1, p2_2;
    //this uses the cam_model_ to project a 3d point
    //to a pixel
    //i'm not using the built-in function, because at least
    //originally the cam_model_ of the kinect had an unusual
    //P matrix... i don't know if they fixed that
    project3dToPixel(p3_1,p2_1);
    project3dToPixel(p3_2,p2_2);
    int this_min_face_size = (int)(floor(fabs(p2_2.x-p2_1.x)));
    ROS_INFO("Minimum face size: %d", this_min_face_size);

    std::vector<cv::Rect> faces_vec;
    cascade_.detectMultiScale(cv_image_gray_, faces_vec,  
			      1.2, 2, CV_HAAR_DO_CANNY_PRUNING, 
			      cv::Size(this_min_face_size,
				       this_min_face_size));

    // Filter the faces using depth information, if available. Currently checks that the actual face size is within the given limits.
    cv::Scalar color(0,255,0);
    Box2D3D one_face;
    double avg_disp = 0.0;
    cv::Mat uvd(1,3,CV_32FC1);
    cv::Mat xyz(1,3,CV_32FC1);
    // For each face...
    for (uint iface = 0; iface < faces_vec.size(); iface++) {//face_seq->total; iface++) {

      one_face.status = "good";
      
      //so... now i want to know
      //what points in the point cloud I have
      //correspond to this box?
      //specifically, is it approximately the correct
      //size and shape for a face?
      one_face.box2d = faces_vec[iface];
      one_face.id = i; // The cascade that computed this face.

      //we take the middle of the box
      //and find the approximate size of it
      
      int startx = floor(one_face.box2d.x + 0.25*one_face.box2d.width) + 1;
      int starty = floor(one_face.box2d.y + 0.25*one_face.box2d.height) + 1;
      int endx = floor(one_face.box2d.x + 0.75*one_face.box2d.width) + 1;
      int endy = floor(one_face.box2d.y + 0.75*one_face.box2d.height) + 1;


      // ROS_INFO("startx = %d, endx = %d, starty = %d, endy = %d", 
      // 	       startx, endx, starty, endy);

      //figure out the approximate dimensions of the face
      //the point cloud is indexed with (0, 0) in the bottom left 
      //corner (often images are indexed with (0, 0) in the
      //top left so this is important to know)
      
      //average depth
      double depth = 0, avgx = 0, avgy = 0; 
      int npts = 0;
      for (int i = startx; i < endx; i++) {
	for (int j = starty; j < endy; j++) {
	  if (!isnan(cloud_(i, j).z)) {
	    avgx += cloud_(i, j).x;
	    avgy += cloud_(i, j).y;
	    depth += cloud_(i, j).z;
	    npts++;
	  }
	}
      }
      if (npts > 0) {
	//we have data!
	avgx /= (double)npts;
	avgy /= (double)npts;
	depth /= (double)npts;
	//figure out the size of the face
	double width = (depth*((double)(endx-startx)))/cam_model_->fx();
	double height = (depth*((double)(endy-starty)))/cam_model_->fy();
	one_face.center3d = cv::Point3d(avgx, avgy, depth);
	one_face.radius3d = (height+width)/2.0;
	//standard deviation of z values
	//things about the size of a face
	//that look like a face but aren't
	//a face, tend to be flatter than a face
	double mean_depth = 100*depth;
	double std_depth = 0;
	for (int i = startx; i < endx; i++) {
	  for (int j = starty; j < endy; j++) {
	    if (!isnan(cloud_(i, j).z)) {
	      std_depth = (100*cloud_(i, j).z - mean_depth)*(100*cloud_(i, j).z - mean_depth); 
	    }
	  }
	}
	std_depth /= (double)npts;
	std_depth = sqrt(std_depth);
	if (one_face.center3d.z > max_face_z_m_ || 
	    //this doesn't work as well as might be hoped
	    //fabs(std_depth) < FACE_SIZE_MIN_DEV_CM ||
      	    2.0*one_face.radius3d < face_size_min_m_ || 
      	    2.0*one_face.radius3d > face_size_max_m_) {
      	  one_face.status = "bad";
      	}
	ROS_INFO("height = %lf, width = %lf, depth = %lf, radius = %lf, std_depth = %lf, avgx = %lf, avgy = %lf, startx = %d, endx=%d, starty=%d, endy=%d",
		 height, width, depth, one_face.radius3d, std_depth, avgx, avgy, startx, endx, starty, endy);
      } else {
      	one_face.radius3d = 0.0;     
      	one_face.center3d = cv::Point3d(0.0,0.0,0.0);
      	one_face.status = "unknown";
      }
      
      // ROS_INFO("ul = (%lf %lf %lf), ur = (%lf %lf %lf) ll = (%lf %lf %lf) lr = (%lf %lf %lf)", ul.x, ul.y, ul.z, ur.x, ur.y, ur.x, 
      // 	       ll.x, ll.y, ll.z, lr.x, lr.y, lr.z);

      // if (ul.z < 1e8 && ur.z < 1e8 && ll.z < 1e8 && lr.z < 1e8
      // 	  && mid.z < 1e8) {
      // 	one_face.center3d = cv::Point3d(mid.x, mid.y, mid.z);
      // 	one_face.radius3d = (fabs(ul.x - ur.x) + fabs(ll.x - lr.x) +
      // 			     fabs(ul.y - ll.y) + fabs(ur.y - lr.y))/4.0;
      // 	ROS_INFO("Center = (%lf, %lf %lf), radius = %lf, max_face_z_m = %lf face_size_min_m = %lf face_size_max_m = %lf",
      // 		 one_face.center3d.x, one_face.center3d.y, one_face.center3d.z,
      // 		 one_face.radius3d,
      // 		 max_face_z_m_, face_size_min_m_, face_size_max_m_);
      //  	if (one_face.center3d.z > max_face_z_m_ || 
      // 	    2.0*one_face.radius3d < face_size_min_m_ || 
      // 	    2.0*one_face.radius3d > face_size_max_m_) {
      // 	  one_face.status = "bad";
      // 	}
      //} else {
      	// one_face.radius3d = 0.0;     
      	// one_face.center3d = cv::Point3d(0.0,0.0,0.0);
      	// one_face.status = "unknown";
	//}
      
      //Add faces to the output vector.
      //lock faces
      boost::mutex::scoped_lock lock(face_mutex_);
      faces_.push_back(one_face);
      lock.unlock();
    }

    boost::mutex::scoped_lock fdmlock(face_done_mutex_);
    num_threads_to_wait_for_--;
    fdmlock.unlock();
    face_detection_done_cond_.notify_one();
  }
}

void Faces::project3dToPixel(const cv::Point3d &xyz,
			     cv::Point2d &uv_rect) const {
  uv_rect.x = (cam_model_->fx()*xyz.x + cam_model_->Tx()) / 
    xyz.z + cam_model_->cx();
  uv_rect.y = (cam_model_->fy()*xyz.y + cam_model_->Ty()) / 
    xyz.z + cam_model_->cy();
}
};
