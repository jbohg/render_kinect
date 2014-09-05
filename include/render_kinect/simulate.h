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

// for publishing TF frames
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

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
	   std::string room_path = "", 
	   const Eigen::Affine3d &room_tf = Eigen::Affine3d()) 
    : out_path_("/tmp/") 
      ,  priv_nh_("~")
      ,  frame_id_( cam_info.frame_id_)
      ,  room_path_(room_path)
      ,  room_tf_(room_tf)
      {
	// allocate memory for depth image
	int w = cam_info.width_;
	int h = cam_info.height_;

	depth_im_ = cv::Mat(h, w, CV_32FC1);
	scaled_im_ = cv::Mat(h, w, CV_8UC1);

	object_models_ = new KinectSimulator(cam_info, 
					     object_paths, 
					     dot_path, 
					     background, 
					     room_path,
					     room_tf);

	// initialize all publishers
	it_ = new image_transport::ImageTransport(priv_nh_);
	pub_depth_image_ = it_->advertise ("depth/image", 5);
	pub_cam_info_ = priv_nh_.advertise<sensor_msgs::CameraInfo > ("depth/camera_info", 5);
	pub_point_cloud_ = priv_nh_.advertise<sensor_msgs::PointCloud2> ("depth/points", 5);
	
	vis_pub = priv_nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

      }

    ~Simulate() {
      delete object_models_;
    }

    void setRoomTransform(Eigen::Affine3d &room_tf)
    {
      object_models_->setRoomTransform(room_tf);
    }

    void setOriginalTransform(const std::vector<Eigen::Affine3d> &part_mesh_transforms)
    {
      object_models_->setOriginalTransform(part_mesh_transforms);
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
      object_models_->intersect(transforms_, point_cloud_, rgb_vals_, depth_im_, labels_);
      
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

    void simulatePublishMeasurement(const std::vector<Eigen::Affine3d> &new_tfs,
				    ros::Time jnt_stamp) {
      countf++;
      
      // update old transform
      transforms_ = new_tfs;

      // simulate measurement of object and store in image, point cloud and labeled image
      cv::Mat p_result;
      object_models_->intersect(transforms_, point_cloud_, rgb_vals_, depth_im_, labels_);

      double min_val, max_val;
      minMaxLoc(rgb_vals_, &min_val, &max_val);

      //      std::cout << "Min and Max value of rgb " << min_val << " " << max_val << std::endl;

      // in case object is not in view, don't store any data
      // However, if background is used, there will be points in the point cloud
      // although they don't belong to the object
      int n_vis = 4000;
      if(point_cloud_.rows<n_vis) {
	std::cout << "Object not in view.\n";
	return;
      }

      // get the current time for synchronisation of all messages
      // ros::Time time = ros::Time::now ();
      // use the time of the joints instead

      // publish TF frames
      publishTransforms(jnt_stamp);

      // publish marker for background
      // publishMarker(jnt_stamp);

      // publish camera info
      publishCameraInfo(jnt_stamp);

      // publish depth image
      publishDepthImage(jnt_stamp);

      // publish point cloud
      publishPointCloud(jnt_stamp);

    }

  private:

    void publishCameraInfo (ros::Time time)
    {
      if (pub_cam_info_.getNumSubscribers () > 0)
	pub_cam_info_.publish (object_models_->getCameraInfo (time));
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

      cv::Mat depth_im_32f; // we have to convert the image to the appropriate encoding
      depth_im_.convertTo(depth_im_32f, CV_32FC1);

      cv_bridge::CvImage cv_img;
      cv_img.header.stamp = time;
      cv_img.header.frame_id = frame_id_;
      cv_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      cv_img.image = depth_im_32f;




//      double min_val, max_val;
//      minMaxLoc(depth_im_32f, &min_val, &max_val);
//      std::cout << "min: " << min_val << "   max: " << max_val << std::endl;
//      cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//      cv::imshow( "Display window", depth_im_32f );                   // Show our image inside it.
//      cv::waitKey(0);
//      exit(-1);

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
      points->fields.resize( 3+1 );
      points->fields[0].name = "x"; 
      points->fields[1].name = "y"; 
      points->fields[2].name = "z";
      points->fields[3].name = "rgb"; 
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

	const float* rgb = rgb_vals_.ptr<float>(i);
	memcpy (&points->data[ i * points->point_step + points->fields[3].offset], 
		&rgb[0], sizeof (float));
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
      marker.id = 10;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = room_tf_.translation()[0];
      marker.pose.position.y = room_tf_.translation()[1];
      marker.pose.position.z = room_tf_.translation()[2];
      
      Eigen::Quaterniond q = (Eigen::Quaterniond)room_tf_.linear();

      marker.pose.orientation.w = q.w();
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.a = 0.5;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://render_kinect/obj_models/room0.obj";
      vis_pub.publish( marker );
    }

    void publishTransforms(ros::Time time)
    {
      // for naming
      std::stringstream obj_frame_id;
      int count = 0;

      tf::Transform transform;
      std::vector<Eigen::Affine3d>::const_iterator it;
      for(it=transforms_.begin(); it!=transforms_.end(); ++it)
	{
	  obj_frame_id.str(std::string());
	  obj_frame_id << "OBJECT" << std::setw(3) << std::setfill('0') << count;
	  count++;

	  tf::transformEigenToTF(*it, transform);
	  ground_truth_.sendTransform(tf::StampedTransform(transform, 
							   time, 
							   frame_id_, 
							   obj_frame_id.str())); 
	}
    }

    void storePointCloud(std::string prefix, 
			 int count)
    {

#ifdef HAVE_PCL
      std::stringstream lD;
      lD << out_path_ << prefix << std::setw(3)
	 << std::setfill('0') << count << ".pcd";

      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      // Fill in the cloud data
      cloud.width = point_cloud_.rows;
      cloud.height = 1;
      cloud.is_dense = false;
      cloud.points.resize(cloud.width * cloud.height);
	
      for (int i = 0; i < point_cloud_.rows; i++) {
	const float* point = point_cloud_.ptr<float>(i);
	const float* rgb = rgb_vals_.ptr<float>(i);
	cloud.points[i].x = point[0];
	cloud.points[i].y = point[1];
	cloud.points[i].z = point[2];
	cloud.points[i].rgb = rgb[0];
      }
	
      if (pcl::io::savePCDFileBinary(lD.str(), cloud) != 0)
	std::cout << "Couldn't store point cloud at " << lD.str() << std::endl;

#else
      std::cout << "Couldn't store point cloud since PCL is not installed." << std::endl;
#endif
    }
    
    KinectSimulator *object_models_;
    cv::Mat depth_im_, scaled_im_, point_cloud_, labels_, rgb_vals_;
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
    tf::TransformBroadcaster ground_truth_;

    // Debugging Visualisation
    std::string room_path_;
    Eigen::Affine3d room_tf_; 
    ros::Publisher vis_pub;
    
  };

} //namespace render_kinect
#endif // SIMULATE_H
