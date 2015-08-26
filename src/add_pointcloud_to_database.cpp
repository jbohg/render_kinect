/*********************************************************************
 *
 *  Copyright (c) 2014, Daniel Kappler - MPI for Intelligent System
 *  (dkappler@tuebingen.mpg.de)
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

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/system/linux_error.hpp>

#include <grasp_database/grasp_database.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <render_kinect/simulate.h>
#include <render_kinect/camera.h>

#include <ros/package.h>

// Mostly reading tools
#include <render_kinect/tools/rosread_utils.h>

#include <iostream>
#include <fstream>

#include <Eigen/Core>

namespace fs = boost::filesystem;
namespace sys = boost::system;

// this is a global variable to have easy debugging
// we do not use static because it will be deprecated
// instead this is supposed to do the trick in cpp
namespace{
  bool g_debug = false;
}


bool Convert_depth_to_point_cloud( const cv::Mat &p_depth_im,
				   const render_kinect::CameraInfo &cam_info, 
				   pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{
  unsigned w = cam_info.width_;
  unsigned h = cam_info.height_;
  float fx = cam_info.fx_;
  float fy = cam_info.fy_;
  float cx = cam_info.cx_;
  float cy = cam_info.cy_;
  float z_near = cam_info.z_near_;
  float z_far  = cam_info.z_far_;
  
  point_cloud.points.clear();
  point_cloud.points.reserve(w*h);
  for (unsigned r = 0; r < h; ++r) {
    const float* depth_map_i = p_depth_im.ptr<float>(r);
    for (unsigned c = 0; c < w; ++c) {
      float d = depth_map_i[c];
      // the maximal depth value is 1.0
      if (d > z_near && d == d && d < z_far) {
	pcl::PointXYZ point;
	point.z = d;
	point.x = (point.z / fx) * (c - cx);
	point.y = (point.z / fy) * (r - cy);
	point_cloud.points.push_back(point);
      }
    }
  }

  point_cloud.width = point_cloud.points.size();
  point_cloud.height = 1;
  point_cloud.is_dense = false;
}


Eigen::Quaterniond Get_rotation(
        int step_x, 
        int step_y,
        float radian_of_rotation_per_step) 
{
  Eigen::Quaterniond result(0, 1, 0, 0);
  Eigen::Quaterniond q_x(Eigen::AngleAxisd(
              step_x*radian_of_rotation_per_step, 
              Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond q_y(Eigen::AngleAxisd(
              step_y*radian_of_rotation_per_step, 
              Eigen::Vector3d::UnitY()));
  result = q_x*result;
  result = q_y*result;
  return result;
}

bool Update_database(
    int argc,
    char** argv,
    const std::string &file_path_database,
    const std::string &object_type,
    int samples_per_half_circle,
    float kinect_distance) {

  grasp_database::Grasp_database grasp_db(file_path_database);

  // render type to distingiush between pure depth 
  // and openGL truncated pyramid style
  std::string render_type = "kinect_sim";

  if(grasp_db.Has_object_pointclouds(object_type, render_type)){
    std::cout << "already generated " << object_type << std::endl;
    return true;
  }

  // we need to store the object file as a file to be able to process it later
  const fs::path TMP_DIR_PATH_OBJECT("/tmp/object_renderer");
  sys::error_code ec;
  fs::create_directory(TMP_DIR_PATH_OBJECT, ec);
  if (g_debug) {
    std::cout << "create directory error code " << ec << std::endl;
  }

  const std::string TMP_FILE_PATH_OBJECT = TMP_DIR_PATH_OBJECT.string() + "/"
    + object_type + ".ply";
  if (g_debug) {
    std::cout << "tmp file path object " << TMP_FILE_PATH_OBJECT
          << std::endl;
  }
  if (!grasp_db.Store_object(object_type, TMP_FILE_PATH_OBJECT)) {
    std::cout << "could not store object " << object_type << std::endl;
    return false;
  }

  std::vector<std::string> object_paths;
  object_paths.push_back(TMP_FILE_PATH_OBJECT);

  // get the path to the package
  std::string path = ros::package::getPath("render_kinect");
  // Get the path to the dot pattern
  std::string dot_pattern_path;
  rosread_utils::getDotPatternPath(path, dot_pattern_path);

  //! Camera Parameters
  render_kinect::CameraInfo cam_info;
  cam_info.width_ = 640;
  cam_info.height_ = 480;
  cam_info.cx_ = 320;
  cam_info.cy_ = 240;
  cam_info.z_near_ = 0.5;
  cam_info.z_far_ = 6.0;
  cam_info.fx_ = 580.0;
  cam_info.fy_ = 580.0;
  // baseline between IR projector and IR camera
  cam_info.tx_ = 0.075;
  // set frame_id
  cam_info.frame_id_ = "XTION";
  // set noise type
  cam_info.noise_.type_ = render_kinect::PERLIN;
  cam_info.noise_.scale_ = 0.2;
    

  //! Test Transform
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());

  //! Kinect Simulator
  render_kinect::Simulate Simulator(cam_info, 
                    object_paths,
                    dot_pattern_path);

  cv::Mat depth_image;

  Eigen::Vector3f viewpoint_position;
  Eigen::Quaternionf object_orientation;
  std::vector<Eigen::Vector3f> object_positions;
  std::vector<Eigen::Quaternionf> object_orientations;
  std::vector<cv::Mat> depth_images;

  std::vector<float> camera_height;
  std::vector<float> camera_width;
  std::vector<float> camera_cx;
  std::vector<float> camera_cy;
  std::vector<float> camera_z_near;
  std::vector<float> camera_z_far;
  std::vector<float> camera_fx;
  std::vector<float> camera_fy;

  float degree_of_rotation_per_step = M_PI/samples_per_half_circle;

  std::vector<Eigen::Affine3d> transforms;

  for (int i = 0; i < 2 * samples_per_half_circle; ++i) {
    for (int j = 0; j < 2 * samples_per_half_circle; ++j) {
      camera_width.push_back(cam_info.width_);
      camera_height.push_back(cam_info.height_);
      camera_cx.push_back(cam_info.cx_);
      camera_cy.push_back(cam_info.cy_);
      camera_z_near.push_back(cam_info.z_near_);
      camera_z_far.push_back(cam_info.z_far_);
      camera_fx.push_back(cam_info.fx_);
      camera_fy.push_back(cam_info.fy_);
      // sample noisy transformation around initial one
      transform = Get_rotation(i,j,degree_of_rotation_per_step);
      transform.translation() = Eigen::Vector3d(0.0, 0.0, kinect_distance);
      transforms.clear();
      transforms.push_back(transform);

      // give pose and object name to renderer
      Simulator.simulateMeasurement(transforms, depth_image);

      pcl::PointCloud<pcl::PointXYZ> point_cloud;
      Convert_depth_to_point_cloud(depth_image, cam_info, point_cloud);
      // if the pointcloud does not contain more than 30 points
      // we skip this image
      if(point_cloud.points.size()<30) {
        std::cout << "could not convert pointcloud with at least "
                  << "30 points so we skip this view" << std::endl;
        continue;
      }
      depth_images.push_back(depth_image);
            
      // we do not need to change the orientation
      // because the viewpoint orientation does never change
      object_positions.push_back(transform.translation().cast<float>());
      object_orientations.push_back(
              Eigen::Quaternionf(transform.rotation().cast<float>()));
    }
  }
    
  grasp_db.Set_object_pointclouds(object_type,
                 render_type,
                 object_positions,
                 object_orientations,
                 depth_images,
                 camera_width,
                 camera_height,
                 camera_cx,
                 camera_cy,
                 camera_z_near,
                 camera_z_far,
                 camera_fx,
                 camera_fy);
    
  return true;
}

int main(int argc, char **argv) {

  // initialize ros
  ros::init(argc, argv, "add_pointclouds_to_database");

  try {
    std::string application_name = boost::filesystem::basename(argv[0]);

    /** Define and parse the program options
     */
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("file-path-database",
       po::value<std::string>()->required(),
       "file path to the database")
      ("object-type",
       po::value<std::string>()->required(),
       "object name")
      ("samples-half-circle,s",
       po::value<int>()->required(),
       "number of samples on 180 degree")
      ("kinect-distance,k",
       po::value<float>()->required(),
       "distance to the kinect sensor")
      ("debug,d",
       po::value<bool>()->default_value(false),
       "debug output");

    po::variables_map vm;
    po::positional_options_description positionalOptions;

    try {
      po::store(
        po::command_line_parser(argc, argv).options(desc).positional(
                                         positionalOptions).run(), vm);
      /** --help option
       */
      if (vm.count("help")) {
        std::cout << "point clouds from different views for an object " 
        << "stored in the database"
        << std::endl 
        << std::endl 
        << desc 
        << std::endl
        << std::endl;
        return 0;
      }

      po::notify(vm); // throws on error, so do after help in case
      // there are any problems
    } catch (boost::program_options::required_option& e) {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl << desc
                << std::endl;
      return -1;
    } catch (boost::program_options::error& e) {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl << desc
                << std::endl;
      return -1;
    }
    // global variable
    g_debug = vm["debug"].as<bool>();
    if (!Update_database(argc, argv,
                         vm["file-path-database"].as<std::string>(),
                         vm["object-type"].as<std::string>(),
                         vm["samples-half-circle"].as<int>(),
                         vm["kinect-distance"].as<float>())) {
      std::cout << "could not update database" << std::endl;
      return -1;
    }

  } catch (std::exception& e) {
    std::cerr << "Unhandled Exception reached the top of main: " << e.what()
          << ", application will now exit" << std::endl;
    return -1;

  }
  return 0;
}
