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
 * Program publishes simulated observations from an object moving with
 * brownian motion.
 */

#include <ros/package.h>

#include <boost/filesystem.hpp>


#include <render_kinect/simulate.h>
#include <render_kinect/random_process.h>
#include <render_kinect/wiener_process.h>
#include <render_kinect/camera.h>

#include <pose_tracking_interface/utils/tracking_dataset.hpp>
#include <pose_tracking/states/free_floating_rigid_bodies_state.hpp>
#include <fast_filtering/utils/helper_functions.hpp>
#include <fast_filtering/models/process_models/interfaces/stationary_process_model.hpp>
#include <pose_tracking/models/process_models/brownian_object_motion_model.hpp>


// Mostly reading tools
#include <render_kinect/tools/rosread_utils.h>

#include <math.h>

#include <ros/time.h>


using namespace std;
using namespace Eigen;
using namespace ff;

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
                                           0.5,
                                           0.5,
                                           0,
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
    ros::NodeHandle node_handle("~");

    // get the path to the package
    std::string path = ros::package::getPath("render_kinect");

    // the path of our dataset will be the same as the config file
    string config_file; node_handle.getParam("config_file", config_file);
    boost::filesystem::path path_dataset = config_file;
    path_dataset = path_dataset.parent_path();

    // the object model has to be inside our dataset folder
    std::vector<std::string> object_filenames; node_handle.getParam("object_file_names", object_filenames);
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
    rosread_utils::getRoomTransform(node_handle, path, room_tf);

    // Get the camera info
    render_kinect::CameraInfo cam_info;
    rosread_utils::getCameraInfo(node_handle, cam_info);

    render_kinect::Simulate Simulator(cam_info,
                                      object_mesh_paths,
                                      dot_pattern_path,
                                      rosread_utils::renderBackground(node_handle),
                                      room_path,
                                      room_tf);



    // Initial Transform for objects
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.translate(Eigen::Vector3d(0., 0., 1.3));
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




    // create process model
    double linear_acceleration_sigma;
    node_handle.getParam("linear_acceleration_sigma", linear_acceleration_sigma);
    double angular_acceleration_sigma;
    node_handle.getParam("angular_acceleration_sigma", angular_acceleration_sigma);
    double damping;
    node_handle.getParam("damping", damping);
    vector<double> rotation_center;
    node_handle.getParam("rotation_center", rotation_center);




    MatrixXd linear_acceleration_covariance = MatrixXd::Identity(3, 3) * pow(double(linear_acceleration_sigma), 2);
    MatrixXd angular_acceleration_covariance = MatrixXd::Identity(3, 3) * pow(double(angular_acceleration_sigma), 2);

//    vector<boost::shared_ptr<StationaryProcess<> > > partial_process_models(object_mesh_paths.size());
//    for(size_t i = 0; i < partial_process_models.size(); i++)
//    {
//        boost::shared_ptr<BrownianObjectMotionModel<> > partial_process_model(new BrownianObjectMotionModel<>);
//        partial_process_model->parameters(hf::Std2Eigen(rotation_center),
//                                          damping,
//                                          linear_acceleration_covariance,
//                                          angular_acceleration_covariance);
//        partial_process_models[i] = partial_process_model;
//    }
//    boost::shared_ptr<StationaryProcess<> > process_model(new ComposedStationaryProcessModel(partial_process_models));
    boost::shared_ptr<BrownianObjectMotionModel<FreeFloatingRigidBodiesState<-1>, -1> >
            process_model(new BrownianObjectMotionModel<FreeFloatingRigidBodiesState<-1>, -1>(object_mesh_paths.size()));
    for(size_t i = 0; i < object_mesh_paths.size(); i++)
    {
        process_model->Parameters(i,
                               hf::Std2Eigen(rotation_center),
                               damping,
                               linear_acceleration_covariance,
                               angular_acceleration_covariance);
    }




    double max_ratio = tan(0.75/2.); // ratio x/z such that the object is guaranteed to be inside of image
    double desired_depth; node_handle.getParam("desired_depth", desired_depth);

    FreeFloatingRigidBodiesState<-1> initial_state(object_mesh_paths.size());
    for(size_t i = 0; i < initial_state.body_count(); i++)
    {
        size_t size = std::ceil(std::sqrt(double(initial_state.body_count())));
        double delta = 2*max_ratio / double(size);
        initial_state.position(i) = Vector3d(-max_ratio + (i%size + 0.5)*delta,
                                             -max_ratio + (i/size + 0.5)*delta,
                                             1.0) * desired_depth;
        initial_state.euler_vector(i) = Vector3d(0.7, 0.7, 0.7);
    }

    cout << "depth " << desired_depth << endl;


    TrackingDataset dataset(path_dataset.string());

    int frame_count; node_handle.getParam("frame_count", frame_count);
    double d_gain; node_handle.getParam("d_gain", d_gain);

    cout << "frame count: " << frame_count << " rendering frame: " << endl;

    FreeFloatingRigidBodiesState<-1> state = initial_state;
    for(int i = 0; i < frame_count && ros::ok(); ++i)
    {
        VectorXd control(VectorXd::Zero(process_model->NoiseDimension()));
        for(size_t j = 0; j < state.body_count(); j++)
        {
            control.middleRows<3>(j*6) = - d_gain * (state.position(j) - initial_state.position(j));
        }

        process_model->Condition(1./30., state, control);
        state = process_model->Sample();

        sensor_msgs::ImagePtr image;
        sensor_msgs::CameraInfoPtr camera_info;

        std::vector<Eigen::Affine3d> object_poses;
        for(size_t j = 0; j < state.body_count(); j++)
            object_poses.push_back(Affine3d(state.homogeneous_matrix(j)));
            
        Simulator.simulateMeasurement(object_poses, image, camera_info);

        dataset.AddFrame(image, camera_info, state.poses());

        cout << i << ", " << flush;
    }
    cout << endl;
    if(dataset.Size() == frame_count)
        dataset.Store();

    return 0;
}
