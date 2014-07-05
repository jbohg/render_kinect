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
/* Header file for camera class that keeps the camera parameters and type of noise.
 * Most important functionality is the conversion of pixel coordinates to rays and 
 * the projection of rays onto the image plane. These functions are taken from the 
 * ros-package image_geometry.
 */

#ifndef KINECT_SIM_CAMERA_H_
#define KINECT_SIM_CAMERA_H_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <boost/make_shared.hpp>

namespace render_kinect
{

  enum NoiseType
  {
    GAUSSIAN=1,
    PERLIN,
    SIMPLEX,
    NONE
  };
  
  class CameraInfo
  {
  public:

    int width_, height_;
    double z_near_, z_far_;
    double fx_, fy_;
    double cx_, cy_;
    double tx_;

    NoiseType noise_;

    std::string frame_id_;
  };

  class Camera
  {
  public:
    
    double getFx()
    {
      return info_.fx_;
    }

    double getFy()
    {
      return info_.fy_;
    }
    
    double getCx()
    {
      return info_.cx_;
    }

    double getCy()
    {
      return info_.cy_;
    }

    int getWidth()
    {
      return info_.width_;
    }

    int getHeight()
    {
      return info_.height_;
    }

    double getZNear()
    {
      return info_.z_near_;
    }

    double getZFar()
    {
      return info_.z_far_;
    }

    double getTx() 
    {
      return info_.tx_;
    }

    
  Camera(const CameraInfo p_info)
    : info_(p_info) {}
    
    // two functions adopted from ros::image_geometry::PinholeCameraModel
    cv::Point2d project3dToPixel(const cv::Point3d& xyz) const
      {
	
	// [U V W]^T = P * [X Y Z 1]^T
	// u = U/W
	// v = V/W
	cv::Point2d uv_rect;
	uv_rect.x = (info_.fx_*xyz.x) / xyz.z + info_.cx_;
	uv_rect.y = (info_.fy_*xyz.y) / xyz.z + info_.cy_;
	return uv_rect;
      }

    cv::Point3d projectPixelTo3dRay(const cv::Point2d& uv_rect) const
      {
	cv::Point3d ray;
	ray.x = (uv_rect.x - info_.cx_) / info_.fx_;
	ray.y = (uv_rect.y - info_.cy_) / info_.fy_;
	ray.z = 1.0;
	return ray;
      }

    sensor_msgs::CameraInfoPtr getCameraInfo (ros::Time time)
      {
	sensor_msgs::CameraInfoPtr info_msg = boost::make_shared<sensor_msgs::CameraInfo > ();
	info_msg->header.stamp    = time;
	info_msg->header.frame_id = info_.frame_id_;
	info_msg->width           = info_.width_;
	info_msg->height          = info_.height_;

	info_msg->D = std::vector<double>(5, 0.0);
	info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	info_msg->K.assign (0.0);
	info_msg->R.assign (0.0);
	info_msg->P.assign (0.0);
	// Simple camera matrix: square pixels, principal point at center
	info_msg->K[0] = info_msg->K[4] = info_.fx_;
	info_msg->K[2] = info_.cx_;
	info_msg->K[5] = info_.cy_;
	info_msg->K[8] = 1.0;
	// no rotation: identity
	info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
	// no rotation, no translation => P=K(I|0)=(K|0)
	info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
	info_msg->P[2] = info_msg->K[2];
	info_msg->P[6] = info_msg->K[5];
	info_msg->P[10] = 1.0;

	return info_msg;
      }

  private:
    CameraInfo info_;
  };
 
} // namespace render_kinect

#endif // KINECT_SIM_CAMERA_H_
