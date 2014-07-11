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
 * brownian motion.
 */

#include <ros/package.h>

#include <boost/filesystem.hpp>


#include <render_kinect/simulate.h>
#include <render_kinect/random_process.h>
#include <render_kinect/wiener_process.h>
#include <render_kinect/camera.h>

#include <state_filtering/tools/tracking_dataset.hpp>
#include <state_filtering/system_states/floating_body_system.hpp>

// Mostly reading tools
#include <render_kinect/tools/rosread_utils.h>


using namespace std;

// Generate a process models for each object instance in the scene
void getProcessModels(int n_objects, 
                      Eigen::Affine3d &init_transform,
                      double var_x,
                      double var_y,
                      double var_z,
                      double var_angle,
                      std::vector<render_kinect::WienerProcess> &object_processes)
{
    // create a Random Process to sample
    // pertubation around initial transform
    render_kinect::RandomProcess rand_proc(init_transform,
                                           var_x,
                                           var_y,
                                           var_z,
                                           var_angle);

    for(int i=0; i<n_objects; ++i)
    {
        Eigen::Affine3d transform;
        rand_proc.getNextTransform(transform);
        object_processes.push_back( render_kinect::WienerProcess(transform,
                                                                 var_x,
                                                                 var_y,
                                                                 var_z,
                                                                 var_angle));
    }
}


// main function that generated a number of sample outputs for a given object mesh. 
int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "move_object");
    ros::NodeHandle nh("~");

    // get the path to the package
    std::string path = ros::package::getPath("render_kinect");

    // the path of our dataset will be the same as the config file
    string config_file; nh.getParam("config_file", config_file);
    boost::filesystem::path path_dataset = config_file;
    path_dataset = path_dataset.parent_path();

    // the object model has to be inside our dataset folder
    std::vector<std::string> object_filenames; nh.getParam("object_file_names", object_filenames);
    std::vector<std::string> object_mesh_paths;
    for(size_t object_index = 0; object_index < object_filenames.size(); object_index++)
        object_mesh_paths.push_back((path_dataset / object_filenames[object_index]).string());

    // Get the path to the dot pattern
    std::string dot_pattern_path;
    rosread_utils::getDotPatternPath(path, dot_pattern_path);

    // Get the path of the room (background mesh)
    std::string room_path;
    rosread_utils::getRoomPath(path, room_path);
    Eigen::Affine3d room_tf;
    rosread_utils::getRoomTransform(nh, path, room_tf);

    // Get the camera info
    render_kinect::CameraInfo cam_info;
    rosread_utils::getCameraInfo(nh, cam_info);

    render_kinect::Simulate Simulator(cam_info,
                                      object_mesh_paths,
                                      dot_pattern_path,
                                      rosread_utils::renderBackground(nh),
                                      room_path,
                                      room_tf);

    // Number of samples
    int frames = 100;

    // Initial Transform for objects
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.translate(Eigen::Vector3d(0.089837, -0.137769, 1.549210));
    transform.rotate(Eigen::Quaterniond(0.906614,-0.282680,-0.074009,-0.304411));

    // object state and process model and noise variances
    double translate_var = 0.001;
    double rotate_var = 0.005;
    std::vector<render_kinect::WienerProcess> object_processes;

    getProcessModels((int)object_mesh_paths.size(),
                     transform,
                     translate_var,
                     translate_var,
                     translate_var,
                     rotate_var,
                     object_processes);

    TrackingDataset dataset(path_dataset.string());
    ROS_INFO("Rendering frame: ");
    for(int i=0; i<frames; ++i)
    {
        // get the next state of object according to the chosen process model
        std::vector<Eigen::Affine3d> object_poses(object_processes.size());
        for(std::vector<render_kinect::WienerProcess>::iterator it=object_processes.begin(); it!=object_processes.end(); ++it)
            it->getNextTransform(object_poses[it-object_processes.begin()]);

        sensor_msgs::ImagePtr image;
        sensor_msgs::CameraInfoPtr camera_info;
        Simulator.simulateMeasurement(object_poses, image, camera_info);

        FloatingBodySystem<-1> state(object_poses.size());
        for(size_t i = 0; i < object_poses.size(); i++)
            state.pose(object_poses[i], i);

        dataset.addFrame(image, camera_info, state.poses());

        ROS_INFO("%d, ", i);
    }
    ROS_INFO("\n");
    dataset.stOre();

    return 0;
}
