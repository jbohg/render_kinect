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
/* Header file for kinect simulator that implements the simulated kinect measurement 
 * given camera parameters and an object model. More info in the cpp file.
 */

#ifndef KINECTSIMULATOR_H_
#define KINECTSIMULATOR_H_

#include <boost/shared_ptr.hpp>

#include <render_kinect/objectMeshModel.h>
#include <render_kinect/camera.h>
#include <render_kinect/noise.h>

#include <opencv2/highgui.hpp>

inline double abs(Point p)
{
  return std::sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
}

inline double sq(float x)
{
  return x*x;
}


namespace render_kinect
{
  class KinectSimulator
  {
  private:

    boost::shared_ptr<ObjectMeshModel> model_;
    TreeAndTri* search_;

    Camera camera_;
    //static const float invalid_disp_ = 99999999.9;
    //static const float window_inlier_distance_ = 0.1;
    static constexpr float invalid_disp_ = 99999999.9;
    static constexpr float window_inlier_distance_ = 0.1;

    void initRobot();
    void updateObjectPoses(const Eigen::Affine3d &p_transform);
    void updateTree();
    void filterDisp(const cv::Mat &disp, const cv::Mat &labels, cv::Mat &out_disp, cv::Mat& out_labels);
    
    // filter masks 
    static const int size_filt_ = 9;
    cv::Mat weights_;
    cv::Mat fill_weights_;
    
    // noise generator
    Noise* noise_gen_;
    // what noise to use on disparity map
    NoiseType noise_type_;
    
    // kinect dot pattern
    std::string dot_path_;
    cv::Mat dot_pattern_;    

    // wether label image should overlap with the noisy depth image
    bool noisy_labels_;

  public:
    
    std::vector<cv::Scalar> color_map_;
    static const uchar background_ = 60;

    uchar getBG ()const{return background_;}

    void intersect(const Eigen::Affine3d &p_transform,//tf::Transform &p_transform, 
		   cv::Mat &point_cloud,
		   cv::Mat &depth_map,
		   cv::Mat &labels);

    KinectSimulator(const CameraInfo &p_camera_info,
		    std::string object_name,
		    std::string dot_path);
    ~KinectSimulator();

   
  };
}//namespace render_kinect

#endif /* KINECTSIMULATOR_H_ */
