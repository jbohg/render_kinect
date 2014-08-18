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

/* move_robot.cpp 
 * Program publishes simulated measurements from a robot moving the joints 
 * as published on /joint_states topic
 */

#include <ros/package.h>

#include <render_kinect/simulate.h>
#include <render_kinect/camera.h>
#include <render_kinect/robot_state.h>

// Mostly reading tools
#include <render_kinect/tools/rosread_utils.h>

namespace render_kinect
{
  class RobotObserver
  {
  public:
    RobotObserver() :
      nh_priv_("~") ,
      noisy_(false)
    {
      // get the path to the package
      std::string path = ros::package::getPath("render_kinect");
      
      // get the relevant parameters
      nh_priv_.param<bool>("noisy", noisy_, false);
      
      std::string joint_states_topic;
      nh_priv_.param<std::string>("joint_states_topic", joint_states_topic, "/joint_states");

      // Get the path to the dot pattern
      std::string dot_pattern_path;
      rosread_utils::getDotPatternPath(path, dot_pattern_path);
      
      // Get the path of the room (background mesh)
      std::string room_path;
      rosread_utils::getRoomPath(path, room_path);
      Eigen::Affine3d room_tf;
      rosread_utils::getRoomTransform(nh_priv_, path, room_tf);
      
      // Get the camera info
      render_kinect::CameraInfo cam_info;
      rosread_utils::getCameraInfo(nh_priv_, cam_info);
      
      // Create robot state that is responsible for converting joint angles to tfs
      robot_state_ = new RobotState( );
      
      // Get the paths to the part_mesh_models
      std::vector<std::string> part_mesh_paths;
      robot_state_->GetPartMeshPaths(part_mesh_paths);

      // Initialize the kinect simulator
      simulator_ = new Simulate(cam_info, 
				part_mesh_paths, 
				dot_pattern_path, 
				rosread_utils::renderBackground(nh_priv_),
				room_path,
				room_tf);

      joint_states_sub_ = nh_priv_.subscribe<sensor_msgs::JointState>(joint_states_topic, 
								      1,
								      &RobotObserver::jointStateCallback, 
								      this);

    }
    
    ~RobotObserver()
    {
      delete robot_state_;
      delete simulator_;
    }

  private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      ros::Time jnt_stamp = msg->header.stamp;
      joint_state_ = *msg;
     
      // TODO: //////////////////////////////////////////////////////////////////////
      // Generate noisy joint_state from which the observations will be generated. //
      // This will then be the true joint state that has to be estimated.          //
      // The joint state that comes in to this function as a message is considered //
      // as the noisy joint angles as read from erroneous encoders etc             //
      ///////////////////////////////////////////////////////////////////////////////
 
      robot_state_->GetTransforms(joint_state_, current_tfs_, false);
      simulator_->simulatePublishMeasurement(current_tfs_);
    }

    ros::NodeHandle nh_priv_;
    ros::Subscriber joint_states_sub_;
    
    RobotState *robot_state_;
    Simulate *simulator_;

    sensor_msgs::JointState joint_state_;

    std::vector<Eigen::Affine3d> current_tfs_;
    bool noisy_;

  }; // class RobotObserver
  
} // namespace render_kinect

// main function that starts ros node that subscribes to joint_states topic
int main(int argc, char **argv)
{
  // initialize ros
  ros::init(argc, argv, "move_robot");
  
  render_kinect::RobotObserver robot_observer;
  ros::spin();

  return 0;
}
