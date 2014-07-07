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

/* move_kinect.cpp 
 * Program publishes simulated measurements from an object moving with 
 * bronian motion.
 */

#include <render_kinect/simulate.h>
#include <render_kinect/random_process.h>
#include <render_kinect/wiener_process.h>
#include <render_kinect/camera.h>
#include <render_kinect/yamlutils.h>

#include <ros/package.h>

#include <iostream>
#include <fstream>

void getDotPatternPath(std::string &pkg_path, 
		       std::string &dot_pattern_path)
{
  std::string dot_path = "/data/kinect-pattern_3x3.png";
  std::stringstream full_dot_path;
  full_dot_path << pkg_path << dot_path;
  dot_pattern_path = full_dot_path.str();
}

void getObjectMeshPath(ros::NodeHandle &nh,
		       std::string &pkg_path,
		       std::string &object_mesh_path)
{
  std::string object_mesh;
  nh.param ("object_mesh", object_mesh, std::string (""));
  if (object_mesh.empty ())
    {
      object_mesh = "wheel.obj";
      ROS_INFO ("'object_mesh' not set. using default: '%s'", object_mesh.c_str());
    }
  else
    ROS_INFO ("object_mesh = '%s' ", object_mesh.c_str());
  

  // Get the path to the object mesh model.
  std::string object_models_dir = "/obj_models/";
  std::stringstream full_path;
  full_path << pkg_path << object_models_dir << object_mesh;
  object_mesh_path = full_path.str();
}

bool renderBackground(ros::NodeHandle &nh)
{
  bool background;
  nh.param ("background", background, false);
  return background;
}

void getRoomPath(std::string &pkg_path,
		 std::string &room_path)
{
  // Get the path to the object mesh model.
  std::string room_mesh = "/obj_models/room0_flipped.obj";
  std::stringstream full_path;
  full_path << pkg_path << room_mesh;
  room_path = full_path.str();
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

  // get the noise model
  std::string noise;
  nh.param ("noise", noise, std::string (""));
  if (noise.empty ()) { 
    noise = "NONE";
    ROS_INFO ("Noise model not set. Using 'NONE'.");
  } else {
    ROS_INFO ("Using noise model '%s'", noise.c_str());
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
      cam_info.noise_ = render_kinect::NONE;
    } else if(noise.compare("GAUSSIAN")==0) {
      cam_info.noise_ = render_kinect::GAUSSIAN;
    } else if (noise.compare("PERLIN")==0) {
      cam_info.noise_ = render_kinect::PERLIN;
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

// main function that generated a number of sample outputs for a given object mesh. 
int main(int argc, char **argv)
{
  // initialize ros
  ros::init(argc, argv, "move_object");
  ros::NodeHandle nh("~");
  
  // get the path to the package
  std::string path = ros::package::getPath("render_kinect");
  
  // get the path to the object mesh
  std::string object_mesh_path;
  getObjectMeshPath(nh, path, object_mesh_path);

  // Get the path to the dot pattern
  std::string dot_pattern_path;
  getDotPatternPath(path, dot_pattern_path);

  // Get the path of the room (background mesh)
  std::string room_path;
  getRoomPath(path, room_path);
  
  // get the camera info
  render_kinect::CameraInfo cam_info;
  getCameraInfo(nh, cam_info);

  // Kinect Simulator
  render_kinect::Simulate Simulator(cam_info, 
				    object_mesh_path, 
				    dot_pattern_path, 
				    renderBackground(nh),
				    room_path);

  // Number of samples
  int frames = 50;

  // Initial Test Transform of Objects
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());
  transform.translate(Eigen::Vector3d(0.089837, -0.137769, 1.949210));
  transform.rotate(Eigen::Quaterniond(0.906614,-0.282680,-0.074009,-0.304411));
  
  // object state and process model and noise variances
  //render_kinect::RandomProcess object_process(transform,0.02,0.02,0.02,0.05);
  render_kinect::WienerProcess object_process(transform,0.01,0.01,0.01,0.05);

  // Storage of random transform
  Eigen::Affine3d current_tf;
  for(int i=0; i<frames; ++i) {
    
    // get the next state of object according to the chosen process model
    object_process.getNextTransform(current_tf);
    
    // give pose and object name to renderer
    Simulator.simulatePublishMeasurement(current_tf);
    
    ROS_INFO("Rendering %d\n", i);
  }

  return 0;
}
