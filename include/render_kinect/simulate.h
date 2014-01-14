/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
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
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
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
/* Header file that sets up the simulator and triggers the simulation 
 * of the kinect measurements and stores the results under a given directory.
 */
#ifndef SIMULATE_H
#define SIMULATE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif 

#include <string.h>

#include <render_kinect/kinectSimulator.h>

static unsigned countf = 0;

namespace render_kinect {

  class Simulate {
  public:
  
  Simulate(CameraInfo &cam_info, std::string object_name, std::string dot_path) 
    : out_path_("/tmp/") 
      {
	// allocate memory for depth image
	int w = cam_info.width;
	int h = cam_info.height;

	depth_im_ = cv::Mat(h, w, CV_32FC1);
	scaled_im_ = cv::Mat(h, w, CV_32FC1);

	object_model_ = new KinectSimulator(cam_info, object_name, dot_path);

	transform_ = Eigen::Affine3d::Identity();

      }

    ~Simulate() {
      delete object_model_;
    }

    void simulateMeasurement(const Eigen::Affine3d &new_tf, bool store_depth, bool store_label, bool store_pcd) {
      countf++;
      
      // update old transform
      transform_ = new_tf;

      // simulate measurement of object and store in image, point cloud and labeled image
      cv::Mat p_result;
      object_model_->intersect(transform_, point_cloud_, depth_im_, labels_);
      
      // in case object is not in view, don't store any data
      // However, if background is used, there will be points in the point cloud
      // although they don't belong to the arm
      int n_vis = 4000;
      if(point_cloud_.rows<n_vis) {
	std::cout << "Object not in view.\n";
	return;
      }

      // store on disk
      if (store_depth) {
	std::stringstream lD;
	lD << out_path_ << "depth_orig" << std::setw(3) << std::setfill('0')
	   << countf << ".png";
	convertScaleAbs(depth_im_, scaled_im_, 255.0f);
	cv::imwrite(lD.str().c_str(), scaled_im_);
      }

      // store on disk
      if (store_label) {
	std::stringstream lD;
	lD << out_path_ << "labels" << std::setw(3) << std::setfill('0')
	   << countf << ".png";
	cv::imwrite(lD.str().c_str(), labels_);
      }

      //convert point cloud to pcl/pcd format
      if (store_pcd) {

#ifdef HAVE_PCL
	std::stringstream lD;
	lD << out_path_ << "point_cloud" << std::setw(3)
	   << std::setfill('0') << countf << ".pcd";

	pcl::PointCloud<pcl::PointXYZ> cloud;
	// Fill in the cloud data
	cloud.width = point_cloud_.rows;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (int i = 0; i < point_cloud_.rows; i++) {
	  const float* point = point_cloud_.ptr<float>(i);
	  cloud.points[i].x = point[0];
	  cloud.points[i].y = point[1];
	  cloud.points[i].z = point[2];
	}
	
	if (pcl::io::savePCDFileBinary(lD.str(), cloud) != 0)
	  std::cout << "Couldn't store point cloud at " << lD.str() << std::endl;

#else
	std::cout << "Couldn't store point cloud since PCL is not installed." << std::endl;
#endif
      }
    }

    KinectSimulator *object_model_;
    cv::Mat depth_im_, scaled_im_, point_cloud_, labels_;
    std::string out_path_;
    Eigen::Affine3d transform_; 

  };

} //namespace render_kinect
#endif // SIMULATE_H
