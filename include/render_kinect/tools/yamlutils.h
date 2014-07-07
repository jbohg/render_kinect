/*********************************************************************
*
*  Copyright (c) 2011, Jeannette Bohg - MPI for Intelligent Systems
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
*   * Neither the name of Jeannette Bohg nor the names of KTH
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

#ifndef YAMLUTILS_H
#define YAMLUTILS_H

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace render_kinect
{

  struct Matrix {
    std::string name;
    int rows;
    int cols;
    std::vector<float> data;
  };

  void operator >> (const YAML::Node& node, Matrix& cam_mat) 
  {
    node["rows"] >> cam_mat.rows;
    node["cols"] >> cam_mat.cols;
    node["data"] >> cam_mat.data;
  };

  void operator >> (const YAML::Node& node, std::vector<float>& data)
  {
    for(size_t i=0; i<node.size(); ++i)
      {
	float val;
	node[i] >> val;
	data.push_back(val);
      }
  }; 

  void fillCamMat( const Matrix &cam_mat, double &f, double &cx, double &cy)
  {
    // assert the matrix is 3x3
    assert(cam_mat.rows * cam_mat.cols == 9);

    // compute average of x-FocalLength and y-FoalLength
    f = (cam_mat.data.at(0*cam_mat.rows+0) + cam_mat.data.at(1*cam_mat.rows+1))/2.0f;
    cx = cam_mat.data.at(0*cam_mat.rows+2);
    cy = cam_mat.data.at(1*cam_mat.rows+2);
  }

  void fillTransform( const Matrix &cam_mat, float &tx, float &ty, float &tz, float &roll, float &pitch, float &yaw)
  {
    // assert the transform consists of 6 values
    assert(cam_mat.rows*cam_mat.cols == 6);
    
    tx = cam_mat.data.at(0);
    ty = cam_mat.data.at(1);
    tz = cam_mat.data.at(2);

    // convert from degrees to rad
    roll  = cam_mat.data.at(3) * M_PI/180.0f;
    pitch = cam_mat.data.at(4) * M_PI/180.0f;
    yaw   = cam_mat.data.at(5) * M_PI/180.0f;
  }

}
#endif // YAMLUTILS_H
