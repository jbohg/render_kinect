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

#include <render_kinect/robot_state.h>
#include <boost/random/normal_distribution.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>

RobotState::RobotState()
  : nh_priv_("~")
{
  // Load robot description from parameter server
  std::string desc_string;
  if(!nh_.getParam("robot_description", desc_string))
    ROS_ERROR("Could not get urdf from param server at %s", desc_string.c_str());

  // Initialize URDF object from robot description
  if (!urdf_.initString(desc_string))
    ROS_ERROR("Failed to parse urdf");
 
  // set up kinematic tree from URDF
  if (!kdl_parser::treeFromUrdfModel(urdf_, kin_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return;
  }  

  // setup path for robot description and root of the tree
  nh_priv_.param<std::string>("robot_description_package_path", description_path_, "..");
  nh_priv_.param<std::string>("rendering_root_left", rendering_root_left_, "L_SHOULDER");
  nh_priv_.param<std::string>("rendering_root_right", rendering_root_right_, "R_SHOULDER");
  
  // initialise ratio of joint range as standard deviation
  nh_priv_.param<double>("standard_dev", ratio_std_, 0.01);

  // create segment map for correct ordering of joints
  segment_map_ =  kin_tree_.getSegments();
  boost::shared_ptr<const urdf::Joint> joint;
  joint_map_.resize(kin_tree_.getNrOfJoints());
  lower_limit_.resize(kin_tree_.getNrOfJoints());
  upper_limit_.resize(kin_tree_.getNrOfJoints());
  for (KDL::SegmentMap::const_iterator seg_it = segment_map_.begin(); seg_it != segment_map_.end(); ++seg_it)
    {
      if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
	{
	  joint = urdf_.getJoint(seg_it->second.segment.getJoint().getName().c_str());
	  // check, if joint can be found in the URDF model of the object/robot
	  if (!joint)
	    {
	      ROS_FATAL("Joint '%s' has not been found in the URDF robot model! Aborting ...", joint->name.c_str());
	      return;
	    }
	  // extract joint information
	  if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
	    {
	      joint_map_[seg_it->second.q_nr] = joint->name;
	      lower_limit_[seg_it->second.q_nr] = joint->limits->lower;
	      upper_limit_[seg_it->second.q_nr] = joint->limits->upper;
	    }
	}
    }
  
  nh_priv_.param<std::string>("camera_frame", cam_frame_name_, "XTION");

  // initialise kinematic tree solver
  tree_solver_ = new KDL::TreeFkSolverPos_recursive(kin_tree_);

  // intialise the robot_state publisher
  robot_state_publisher_ = new robot_state_pub::RobotStatePublisher(kin_tree_);
}

RobotState::~RobotState()
{
  delete tree_solver_;
  delete robot_state_publisher_;
}

void RobotState::GetPartMeshData(std::vector<std::string> &part_mesh_paths,
				 std::vector<Eigen::Affine3d> &part_mesh_transform)
{
  //Load robot mesh for each link
  std::vector<boost::shared_ptr<urdf::Link> > links;
  urdf_.getLinks(links);
  std::string global_root =  urdf_.getRoot()->name;
  for (unsigned i=0; i< links.size(); i++)
    {
      // keep only the links descending from our root
      boost::shared_ptr<urdf::Link> tmp_link = links[i];
      while(tmp_link->name.compare(rendering_root_left_)!=0 && 
	    tmp_link->name.compare(rendering_root_right_)!=0 && 
	    tmp_link->name.compare(global_root)!=0)
	{
	  tmp_link = tmp_link->getParent();
	}
      
      if(tmp_link->name.compare(global_root)==0)
	continue;
      
      if ((links[i]->visual.get() != NULL) &&
	  (links[i]->visual->geometry.get() != NULL) && 
	  (links[i]->visual->geometry->type == urdf::Geometry::MESH))
	{ 
	  
	  boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh> (links[i]->visual->geometry);
	  std::string filename (mesh->filename);

	  if (filename.substr(filename.size() - 4 , 4) == ".stl" || filename.substr(filename.size() - 4 , 4) == ".dae")
	    {
	      if (filename.substr(filename.size() - 4 , 4) == ".dae")
		filename.replace(filename.size() - 4 , 4, ".stl");
	      filename.erase(0,25);
	      filename = description_path_ + filename;
	      
	      // if the link has an actual mesh file to read
	      std::cout << "link " << links[i]->name << " is descendant of " << tmp_link->name << std::endl;
	      // store the original transform of the part
	      Eigen::Affine3d tmp;
	      tmp.linear() = Eigen::Quaterniond(links[i]->visual->origin.rotation.w,
						links[i]->visual->origin.rotation.x, 
						links[i]->visual->origin.rotation.y, 
						links[i]->visual->origin.rotation.z).toRotationMatrix(); 
	      tmp.translation() = Eigen::Vector3d(links[i]->visual->origin.position.x, 
						  links[i]->visual->origin.position.y, 
						  links[i]->visual->origin.position.z);
	      part_mesh_transform.push_back(tmp);
	      // stores the filenames
	      part_mesh_paths.push_back(filename);
	      // Produces an index map for the links
	      part_mesh_map_.push_back(links[i]->name);
	    }
	} 
    }
}


void RobotState::GetTransforms(const sensor_msgs::JointState &joint_state, 
			       std::vector<Eigen::Affine3d> &current_tfs)
{
  // compute the link transforms given the joint angles
  InitKDLData(joint_state);

  // get the vector of transformations
  // loop over all segments to compute the link transformation
  current_tfs.clear();
  current_tfs.reserve(frame_map_.size());

  for (KDL::SegmentMap::const_iterator seg_it = segment_map_.begin(); 
       seg_it != segment_map_.end(); ++seg_it)
    {
      if (std::find(part_mesh_map_.begin(), 
		    part_mesh_map_.end(), 
		    seg_it->second.segment.getName()) != part_mesh_map_.end())
	{
	  Eigen::Affine3d tmp;
	  tf::transformKDLToEigen(frame_map_[seg_it->second.segment.getName()], tmp);
	  current_tfs.push_back(tmp);
	}
    }
}



void RobotState::InitKDLData(const sensor_msgs::JointState &joint_state)
{
  // convert joint_state into an EigenVector
  Eigen::VectorXd jnt_angles;
  GetJointVector(joint_state, jnt_angles);
  // Internally, KDL array use Eigen Vectors
  jnt_array_.data = jnt_angles;
  // Given the new joint angles, compute all link transforms in one go
  ComputeLinkTransforms();
}

bool RobotState::GetRoomTransform(Eigen::Affine3d &room_tf)
{
  // get the transform from base to camera
  if(tree_solver_->JntToCart(jnt_array_, cam_frame_, cam_frame_name_)<0)
    ROS_ERROR("TreeSolver returned an error for link %s", cam_frame_name_.c_str());
  cam_frame_ = cam_frame_.Inverse();
  
  tf::transformKDLToEigen(cam_frame_, room_tf);

  return true;
}

void RobotState::ComputeLinkTransforms( )
{
  // get the transform from base to camera
  if(tree_solver_->JntToCart(jnt_array_, cam_frame_, cam_frame_name_)<0)
    ROS_ERROR("TreeSolver returned an error for link %s", cam_frame_name_.c_str());
  cam_frame_ = cam_frame_.Inverse();

  // loop over all segments to compute the link transformation
  for (KDL::SegmentMap::const_iterator seg_it = segment_map_.begin(); 
       seg_it != segment_map_.end(); ++seg_it)
    {
      if (std::find(part_mesh_map_.begin(), 
		    part_mesh_map_.end(), 
		    seg_it->second.segment.getName()) != part_mesh_map_.end())
	{
	  KDL::Frame frame;
	  if(tree_solver_->JntToCart(jnt_array_, frame, seg_it->second.segment.getName())<0)
	    ROS_ERROR("TreeSolver returned an error for link %s", 
		      seg_it->second.segment.getName().c_str());
	  frame_map_[seg_it->second.segment.getName()] = cam_frame_ * frame;
	}
    }
}

void RobotState::GetJointVector(const sensor_msgs::JointState &state, 
				Eigen::VectorXd &jnt_angles)
{
  jnt_angles.resize(num_joints());
  // loop over all joint and fill in KDL array
  for(std::vector<double>::const_iterator jnt = state.position.begin(); 
      jnt !=state.position.end(); ++jnt)
    {
      int tmp_index = GetJointIndex(state.name[jnt-state.position.begin()]);
      
      if (tmp_index >=0) 
	jnt_angles(tmp_index) = *jnt;
      else 
	ROS_ERROR("i: %d, No joint index for %s", 
		  (int)(jnt-state.position.begin()), 
		  state.name[jnt-state.position.begin()].c_str());
    }
  
}

void RobotState::GetJointVector(const sensor_msgs::JointState &state, 
				const ros::Time &time,
				const std::string &tf_prefix,
				std::vector<tf::StampedTransform> &tf_transforms, 
				std::vector<tf::StampedTransform> &fixed_tf_transforms)
{
  // loop over all joint and fill in joint map
  std::map<std::string, double> joint_positions;
  for(std::vector<double>::const_iterator jnt = state.position.begin(); 
      jnt !=state.position.end(); ++jnt)
    joint_positions[state.name[jnt-state.position.begin()]] = *jnt;

  tf_transforms.clear(); 
  fixed_tf_transforms.clear();
  robot_state_publisher_->getTransforms(joint_positions,
					time,
					tf_prefix,
					tf_transforms);
  robot_state_publisher_->getFixedTransforms(time,
					     tf_prefix,
					     fixed_tf_transforms);
  
}

void RobotState::GetNoisyJointVector(const sensor_msgs::JointState &state,
				     sensor_msgs::JointStatePtr &noisy_state,
				     Eigen::VectorXd &noisy_jnt_angles)
{
  // copy old state into noisy state to keep joint names 
  noisy_state = boost::make_shared<sensor_msgs::JointState > (state);
  
  noisy_jnt_angles.resize(num_joints());
  // loop over all joint and fill in KDL array
  for(std::vector<double>::const_iterator jnt = state.position.begin(); 
      jnt !=state.position.end(); ++jnt)
    {
      int tmp_index = GetJointIndex(state.name[jnt-state.position.begin()]);
      
      if (tmp_index >=0) {
	noisy_jnt_angles(tmp_index) = GetRandomPertubation( tmp_index, *jnt, ratio_std_);
	noisy_state->position[jnt-state.position.begin()] = noisy_jnt_angles(tmp_index);	
      } else 
	ROS_ERROR("i: %d, No joint index for %s", 
		  (int)(jnt-state.position.begin()), 
		  state.name[jnt-state.position.begin()].c_str());
    }
}

int RobotState::GetJointIndex(const std::string &name)
{
    for (unsigned int i=0; i < joint_map_.size(); ++i)
      if (joint_map_[i] == name)
	return i;
    return -1;
}

int RobotState::num_joints()
{
  return kin_tree_.getNrOfJoints();
}

double RobotState::GetRandomPertubation(int jnt_index, double jnt_angle, double ratio)
{
  double mean = jnt_angle;
  double range = upper_limit_[jnt_index]-lower_limit_[jnt_index];
  double std  = ratio * range;
  std::normal_distribution<double> normal(mean, std);
  double val = normal(generator_);

  // clip the values to the limits
  if(val>upper_limit_[jnt_index])
    val = upper_limit_[jnt_index];

  if(val<lower_limit_[jnt_index])
    val = lower_limit_[jnt_index];
    
  return val;
}
