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

#include <iostream>
#include <fstream>

#include <render_kinect/tools/yamlutils.h>

namespace rosread_utils {

  void getDotPatternPath(std::string &pkg_path, 
			 std::string &dot_pattern_path)
  {
    std::string dot_path = "/data/kinect-pattern_3x3.png";
    std::stringstream full_dot_path;
    full_dot_path << pkg_path << dot_path;
    dot_pattern_path = full_dot_path.str();
  }

  void getObjectMeshPaths(ros::NodeHandle &nh,
			  std::string &pkg_path,
			  std::vector<std::string> &object_mesh_paths)
  {
    std::vector<std::string> object_meshes;
    if(nh.getParam("object_file_names", object_meshes)) {
      ROS_INFO ("Loading %d object meshes", (int)object_meshes.size());
    } else {
      object_meshes.push_back("wheel.obj");
      ROS_WARN ("'object_mesh' not set. using default: '%s'", object_meshes[0].c_str());
    }

    // Get the path to the object mesh model.
    std::string object_models_dir = "/obj_models/";
    std::stringstream full_path;
    std::vector<std::string>::const_iterator it;
    for(it=object_meshes.begin(); it!=object_meshes.end(); ++it) {
      full_path.str(std::string());
      full_path << pkg_path << object_models_dir << *it;
      object_mesh_paths.push_back(full_path.str());
    }
  }

  bool renderBackground(ros::NodeHandle &nh)
  {
    bool background;
    nh.param("background", background, false);
    return background;
  }

  void getRoomPath(std::string &pkg_path,
		   std::string &room_path)
  {
    // Get the path to the object mesh model.
    std::string room_mesh = "/obj_models/room0.obj";
    std::stringstream full_path;
    full_path << pkg_path << room_mesh;
    room_path = full_path.str();
  }

  void getRoomTransform(ros::NodeHandle &nh,
			std::string &pkg_path,
			Eigen::Affine3d &room_tf)
  {
    room_tf = Eigen::Affine3d::Identity();
    std::vector<float> trans;
    if(nh.getParam("room_trans", trans)) {
      if(trans.size()==3) {
	room_tf.translate(Eigen::Vector3d(trans[0], trans[1], trans[2]));
      } else {
	ROS_ERROR("Room Translation does not have three components in config file.");
	exit(-1);
      }
    }

    std::vector<float> quat;
    if(nh.getParam("room_quat", quat)) {
      if(quat.size()==4) {
	room_tf.rotate(Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]));
      } else {
	ROS_ERROR("Room Quaternion does not have four components in config file.");
	exit(-1);
      }
    }
  }

  void getCameraInfo(ros::NodeHandle &nh,
		     render_kinect::CameraInfo &cam_info)
  {
    std::string frame_id;
    nh.param ("depth_frame_id", frame_id, std::string (""));
    if (frame_id.empty ())
      {
	frame_id = "/camera_depth_optical_frame";
	ROS_INFO ("'frame_id' not set. using default: '%s'", frame_id.c_str());
      }
    else
      ROS_INFO ("frame_id = '%s' ", frame_id.c_str());

    // get the path to the camera parameters
    std::string calib_url;
    bool calibration_valid = false;
    nh.param ("calib_url", calib_url, std::string (""));
    if (calib_url.empty ())
      ROS_INFO ("Calibration file not set. using default values");
    else 
      calibration_valid = true;
    ROS_INFO ("Loading Calibration from '%s'", calib_url.c_str());

    // get the noise model and its parameters
    std::string noise;
    double mean, std, scale;
    nh.param ("noise", noise, std::string (""));
    if (noise.empty ()) { 
      noise = "NONE";
      ROS_INFO ("Noise model not set. Using 'NONE'.");
    } else {
      nh.param("mean", mean, 0.0);
      nh.param("std", std, 0.15);
      nh.param("scale", scale, 0.4);
      ROS_INFO ("Using noise model '%s' with parameters mean=%f, std=%f and scale=%f", 
		noise.c_str(), mean, std, scale);
    }

    // image size
    cam_info.width_ = 640;
    cam_info.height_ = 480;

    // depth limits
    cam_info.z_near_ = 0.5;
    cam_info.z_far_ = 6.0;

    // set frame_id
    cam_info.frame_id_ = frame_id;

    // baseline between IR projector and IR camera
    cam_info.tx_ = 0.075;

    // Type of noise
    if (noise.compare("NONE")==0) {
      cam_info.noise_.type_ = render_kinect::NONE;
    } else if(noise.compare("GAUSSIAN")==0) {
      cam_info.noise_.type_ = render_kinect::GAUSSIAN;
      cam_info.noise_.mean_ = mean;
      cam_info.noise_.std_ = std;
      if(mean==0.0 && std==0.0)
	ROS_WARN("Using zero mean and zero std for Gaussian noise. This seems wrong in the config file.");
    } else if (noise.compare("PERLIN")==0) {
      cam_info.noise_.type_ = render_kinect::PERLIN;
      cam_info.noise_.scale_ = scale;
      if(scale==0.0)
	ROS_WARN("Using zero scale for PERLIN noise. This seems wrong in the config file.");
    } else {
      ROS_ERROR("Given noise type invalid: %s\n", noise.c_str());
      exit(-1);
    }
  
    if (!calibration_valid) {
      // Default parameters
      cam_info.cx_ = 320;
      cam_info.cy_ = 240;
    
      cam_info.fx_ = 580.0;
      cam_info.fy_ = 580.0;
    
    } else {

      // parse the yaml calibration file
      std::ifstream fin(calib_url.c_str());
      if(fin.fail()){
	ROS_ERROR("Could not read calibration file at %s", calib_url.c_str() );
	exit(-1);
      }
      YAML::Parser parser(fin);
      std::string out;
      YAML::Node node;
      parser.GetNextDocument(node);
      for (YAML::Iterator i = node.begin(); i != node.end(); ++i) {
	render_kinect::Matrix cam_mat;
	const YAML::Node & key   = i.first();
	key >> out;
	cam_mat.name = out;
	const YAML::Node & value = i.second();
	value >> cam_mat;
      
	if( out == "camera_matrix_ir" ) {
	  fillCamMat(cam_mat, cam_info.fx_, cam_info.cx_, cam_info.cy_);
	  cam_info.fy_ = cam_info.fx_;
	}
      } // yaml parsing
    } // ifelse calibration file valid
  }

} //namespace
