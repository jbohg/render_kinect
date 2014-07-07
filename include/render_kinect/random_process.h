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
/* Header file that implements the process model of the object.
*/

#ifndef RANDOM_PROCESS_H
#define RANDOM_PROCESS_H

#include <render_kinect/process_model.h>

namespace render_kinect {
  
  class RandomProcess : ProcessModel {
    
  public:
    
    // Constructor that uses standard noise variances
  RandomProcess()
    : ProcessModel()
      {};
    
  RandomProcess( Eigen::Affine3d &p_tf, 
		 double p_x = 0.02,
		 double p_y = 0.02,
		 double p_z = 0.02,
		 double p_angle = 0.05)
    : ProcessModel(p_tf, p_x, p_y, p_z, p_angle)
      {};
    
    // return the next state of the object
    void getNextTransform( Eigen::Affine3d &p_tf )
    {
      Eigen::Vector3d axis(((double)(rand()%1000))/1000.0,
			   ((double)(rand()%1000))/1000.0,
			   ((double)(rand()%1000))/1000.0);
      Eigen::Vector3d t(var_x_*(double)(rand()%2000 -1000)/1000,
			var_y_*(double)(rand()%2000 -1000)/1000,
			var_z_*(double)(rand()%2000 -1000)/1000);
      Eigen::Affine3d noise = Eigen::Affine3d::Identity();
      noise.translate(t);
      noise.rotate(Eigen::AngleAxisd( var_angle_*(double)(rand()%2000 - 1000)/1000, axis));
      p_tf = noise * init_tf_;
    }

  };

} // namespace

#endif // RANDOM_PROCESS_H
