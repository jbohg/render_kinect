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

#ifndef PROCESS_MODEL_H
#define PROCESS_MODEL_H

namespace render_kinect {
  
  class ProcessModel {
    
  public:
    // Constructor with identity as initial 4x4 transform
  ProcessModel()
    : init_tf_(Eigen::Affine3d::Identity())
      , var_x_(0.02)
      , var_y_(0.02)
      , var_z_(0.02)
      , var_angle_(0.05)
      {};
    // Constructor with p_tf as initial 4x4 transform
  ProcessModel( Eigen::Affine3d &p_tf, 
		double p_x = 0.02,
		double p_y = 0.02,
		double p_z = 0.02,
		double p_angle = 0.05)
    : init_tf_(p_tf)
      , var_x_(p_x)
      , var_y_(p_y)
      , var_z_(p_z)
      , var_angle_(p_angle)
    {};
    
    // return the next state of the object
    virtual void getNextTransform( Eigen::Affine3d &p_tf ) = 0;
    
    void setNoiseParameters(const double &p_x,
			    const double &p_y,
			    const double &p_z,
			    const double &p_angle)
    {
      var_x_ = p_x;
      var_y_ = p_y;
      var_z_ = p_z;
      var_angle_ = p_angle;
    }

  protected:
    // initial object position
    Eigen::Affine3d init_tf_;

    // noise variance per dimension
    double var_x_;
    double var_y_;
    double var_z_;
    double var_angle_;
  };

} // namespace

#endif // PROCESS_MODEL_H
