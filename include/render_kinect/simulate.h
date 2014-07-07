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
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif 

#include <string.h>

#include <render_kinect/kinectSimulator.h>

// ROS
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

static unsigned countf = 0;

namespace render_kinect {

  class Simulate {
  public:
  
  Simulate(CameraInfo &cam_info, 
	   std::vector<std::string> &object_paths, 
	   std::string dot_path, 
	   bool background = false,
	   std::string room_path = "") 
    : out_path_("/tmp/") 
      ,  priv_nh_("~")
      ,  frame_id_( cam_info.frame_id_)
      ,  room_path_(room_path)
      {
	// allocate memory for depth image
	int w = cam_info.width_;
	int h = cam_info.height_;

	depth_im_ = cv::Mat(h, w, CV_32FC1);
	scaled_im_ = cv::Mat(h, w, CV_8UC1);

	object_model_ = new KinectSimulator(cam_info, 
					    object_paths, 
					    dot_path, 
					    background, 
					    room_path);

	// initialize all publishers
	it_ = new image_transport::ImageTransport(priv_nh_);
	pub_depth_image_ = it_->advertise ("depth/image", 5);
	pub_cam_info_ = priv_nh_.advertise<sensor_msgs::CameraInfo > ("depth/camera_info", 5);
	pub_point_cloud_ = priv_nh_.advertise<sensor_msgs::PointCloud2> ("depth/points", 5);
	
	vis_pub = priv_nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

      }

    ~Simulate() {
      delete object_model_;
    }

    void simulateStoreMeasurement(const std::vector<Eigen::Affine3d> &new_tfs, 
				  bool store_depth, 
				  bool store_label, 
				  bool store_pcd) {
      countf++;
      
      // update old transform
      transforms_ = new_tfs;

      // simulate measurement of object and store in image, point cloud and labeled image
      cv::Mat p_result;
      object_model_->intersect(transforms_, point_cloud_, depth_im_, labels_);
      
      // in case object is not in view, don't store any data
      // However, if background is used, there will be points in the point cloud
      // although they don't belong to the arm
      int n_vis = 4000;
      if(point_cloud_.rows<n_vis) {
	std::cout << "Object not in view.\n";
	return;
      }

      // store on disk
      if (store_depth)
	storeDepthImage("depth_orig", countf);

      // store on disk
      if (store_label)
	storeLabeledImage("labels", countf);

      //convert point cloud to pcl/pcd format
      if (store_pcd)
	storePointCloud("point_cloud", countf);
    }

    void simulatePublishMeasurement(const std::vector<Eigen::Affine3d> &new_tfs) {
      countf++;
      
      // update old transform
      transforms_ = new_tfs;

      // simulate measurement of object and store in image, point cloud and labeled image
      cv::Mat p_result;
      object_model_->intersect(transforms_, point_cloud_, depth_im_, labels_);      
      
      // in case object is not in view, don't store any data
      // However, if background is used, there will be points in the point cloud
      // although they don't belong to the object
      int n_vis = 4000;
      if(point_cloud_.rows<n_vis) {
	std::cout << "Object not in view.\n";
	return;
      }

      // get the current time for synchronisation of all messages
      ros::Time time = ros::Time::now ();

      // publish marker for background
      //publishMarker(time);
      
      // publish camera info
      publishCameraInfo(time);

      // publish depth image
      publishDepthImage(time);

      // publish point cloud
      publishPointCloud(time);


    }

  private:

    void publishCameraInfo (ros::Time time)
    {
      if (pub_cam_info_.getNumSubscribers () > 0)
	pub_cam_info_.publish (object_model_->getCameraInfo (time));
    }

    void storeDepthImage (std::string prefix, 
			  int count)
    {
      std::stringstream lD;
      lD << out_path_ << prefix << std::setw(3) << std::setfill('0')
	 << count << ".png";
      convertScaleAbs(depth_im_, scaled_im_, 255.0f);
      cv::imwrite(lD.str().c_str(), scaled_im_);
    }

    void storeLabeledImage (std::string prefix, 
			    int count)
    {
      std::stringstream lD;
      lD << out_path_ << prefix << std::setw(3) << std::setfill('0')
	 << count << ".png";
      cv::imwrite(lD.str().c_str(), labels_);
    }

    void publishDepthImage (ros::Time time)
    {
      // DEBUG
      // double min_val, max_val;
      // cv::minMaxLoc(depth_im_, &min_val, &max_val);
      // ROS_INFO("Minimum and Maximum Depth Value: %f %f", min_val, max_val);

      cv_bridge::CvImage cv_img;
      cv_img.header.stamp = time;
      cv_img.header.frame_id = frame_id_;
      cv_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      cv_img.image = depth_im_;

      if (pub_depth_image_.getNumSubscribers () > 0)
	pub_depth_image_.publish (cv_img.toImageMsg());
      
    }
    
    void publishPointCloud(ros::Time time)
    {

      sensor_msgs::PointCloud2Ptr points = boost::make_shared<sensor_msgs::PointCloud2 > ();
      points->header.frame_id = frame_id_;
      points->header.stamp = time;
      points->width        = point_cloud_.rows;
      points->height       = 1;
      points->is_dense     = true;
      points->is_bigendian = false;
      points->fields.resize( 3 );
      points->fields[0].name = "x"; 
      points->fields[1].name = "y"; 
      points->fields[2].name = "z";
      int offset = 0;
      for (size_t d = 0; 
	   d < points->fields.size (); 
	   ++d, offset += sizeof(float)) {
	points->fields[d].offset = offset;
	points->fields[d].datatype = 
	  sensor_msgs::PointField::FLOAT32;
	points->fields[d].count  = 1;
      }

      points->point_step = offset;
      points->row_step   = 
	points->point_step * points->width;
      
      points->data.resize (points->width * 
			   points->height * 
			   points->point_step);
      
      for (int i = 0; i < point_cloud_.rows; i++) {
	const float* point = point_cloud_.ptr<float>(i);
	memcpy (&points->data[i * points->point_step + points->fields[0].offset], 
		&point[0], sizeof (float));
	memcpy (&points->data[ i * points->point_step + points->fields[1].offset], 
		&point[1], sizeof (float));
	memcpy (&points->data[ i * points->point_step + points->fields[2].offset], 
		&point[2], sizeof (float));
      }

      if (  pub_point_cloud_.getNumSubscribers () > 0)
	pub_point_cloud_.publish (points);
    }


    void publishMarker(ros::Time time)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = time;
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.action = visualization_msgs::Marker::ADD;

      /*
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 1.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 1.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.5;
      marker.pose.orientation.w = 1.0;
      */

      marker.pose.position.x = -1.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = -1.0;
      
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.5;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.a = 0.5;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://render_kinect/obj_models/room0_flipped.obj";
      vis_pub.publish( marker );
    }

    void storePointCloud(std::string prefix, 
			 int count)
    {

#ifdef HAVE_PCL
      std::stringstream lD;
      lD << out_path_ << prefix << std::setw(3)
	 << std::setfill('0') << count << ".pcd";

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
    
    KinectSimulator *object_model_;
    cv::Mat depth_im_, scaled_im_, point_cloud_, labels_;
    std::string out_path_;
    std::string frame_id_;
    std::vector<Eigen::Affine3d> transforms_; 

    // node handle
    ros::NodeHandle priv_nh_;

    // topics and publishers
    ros::Publisher pub_cam_info_;
    image_transport::ImageTransport* it_;
    image_transport::Publisher pub_depth_image_;
    ros::Publisher pub_point_cloud_;

    // Debugging Visualisation
    std::string room_path_;
    ros::Publisher vis_pub;
    
  };

} //namespace render_kinect
#endif // SIMULATE_H
