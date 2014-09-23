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


#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <vector>
#include <list>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include <robot_state_pub/robot_state_publisher.h>

#include <tf/tf.h>

// tools
#include <render_kinect/objectMeshModel.h>

class RobotState
{
public:
  
  RobotState();
  ~RobotState();
  
  // outputs a list of mesh model objects and their original transform
  void GetPartMeshData(std::vector<std::string> &part_mesh_paths,
		       std::vector<Eigen::Affine3d> &part_mesh_transform);
  
  void GetTransforms(const sensor_msgs::JointState &joint_state, 
		     std::vector<Eigen::Affine3d> &current_tfs);

  /* void GetTransforms(const sensor_msgs::JointState &joint_state,  */
  /* 		     std::vector<Eigen::Affine3d> &current_tfs, */
  /* 		     std::vector<tf::StampedTransform> &tf_transforms); */

  // convert JointState message to Eigen Vector
  void GetJointVector(const sensor_msgs::JointState &state, 
		      Eigen::VectorXd &jnt_angles);

  // convert joint angles into a joint map
  void GetJointVector(const sensor_msgs::JointState &state, 
		      const ros::Time &time,
		      const std::string &tf_prefix,
		      std::vector<tf::StampedTransform> &tf_transforms, 
		      std::vector<tf::StampedTransform> &fixed_tf_transforms);
  
  // generates a noisy JointState message and returns it in form
  // of a JointState message and Eigen Vector
  void GetNoisyJointVector(const sensor_msgs::JointState &state, 
			   sensor_msgs::JointStatePtr &noisy_state, 
			   Eigen::VectorXd &noisy_jnt_angles);

  // get the transform of the 'background' room based on the cam2base transform
  bool GetRoomTransform(Eigen::Affine3d &room_tf);

private:

  // Initialises the KDL data and specifically the camera pose
  void InitKDLData(const sensor_msgs::JointState &joint_state);
   
  // compute the transformations for all the links in one go
  void ComputeLinkTransforms();

  // Get the index of the joint name in the map
  int GetJointIndex(const std::string &name);
  
  // Get the number of joints from the kinematic tree
  int num_joints();

  double GetRandomPertubation(int jnt_index, double jnt_angle, double ratio);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::string description_path_;

  // model as constructed form the robot urdf description
  urdf::Model urdf_;
  // KDL kinematic tree
  KDL::Tree kin_tree_;
  
  // maps joint indices to joint names and joint limits
  std::vector<std::string> joint_map_;
  std::vector<float> lower_limit_;
  std::vector<float> upper_limit_;

  // maps mesh indices to link names
  std::vector<std::string> part_mesh_map_;
  // maps link names to KDL frames
  std::map<std::string, KDL::Frame> frame_map_;

  // KDL segment map connecting link segments to joints
  KDL::SegmentMap segment_map_;
  // Forward kinematics solver
  KDL::TreeFkSolverPos_recursive *tree_solver_;

  // KDL copy of the joint state
  KDL::JntArray jnt_array_;
  // Contains Camera pose relative to base
  KDL::Frame    cam_frame_;
  std::string   cam_frame_name_;

  // rendering roots for left and right arm to exclude occluding head meshes
  std::string rendering_root_left_, rendering_root_right_;

  // noise ratio relative to joint range
  double ratio_std_;

  // random number generator for getting noisy joint values
  std::mt19937 generator_;

  // for the conversion to tf message
  robot_state_pub::RobotStatePublisher *robot_state_publisher_;
  
};

#endif //ROBOT_STATE_H_
