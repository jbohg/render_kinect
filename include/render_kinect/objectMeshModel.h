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
/* This file implements the object model calss that loads the mesh file, 
 * keeps vertex and triangle information as well as labels.
 * It also uploads this information to the CGAL AABB tree. 
 */
#ifndef _OBJECT_MESH_MODEL_
#define _OBJECT_MESH_MODEL_

#ifdef HAVE_precise
#include "assimp/assimp.h"
#include "assimp/aiPostProcess.h"
#include "assimp/aiScene.h"
#elif defined HAVE_quantal // uses assimp3.0 
#include "assimp/cimport.h"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#else
#include "assimp/cimport.h"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#endif

#include <Eigen/Dense>
#include <Eigen/Core>

#define CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES 1
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/centroid.h>
#include <CGAL/Simple_cartesian.h>

#include <CGAL/intersections.h>
#include <CGAL/Bbox_3.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
typedef K::Vector_3 Vector;
typedef K::Triangle_3 Triangle;
typedef K::Ray_3 Ray;

typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K,Iterator> Primitive;

typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef AABB_triangle_traits::Point_and_primitive_id Point_and_Primitive_id;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef Tree::Object_and_primitive_id Object_and_Primitive_id;

struct TreeAndTri {
  std::vector<K::Triangle_3> triangles;
  std::vector<K::Point_3> points;
  std::vector<int> part_ids;
  Tree tree;
};

namespace render_kinect
{
  // Class that stores all the object geometry information and transformation
  class ObjectMeshModel
  {
  public:
    // Constructor that loads the geometry from a given file name
    ObjectMeshModel(const std::string object_path)
      {
	scene_ =  aiImportFile(object_path.c_str(),aiProcessPreset_TargetRealtime_Quality);
	if(scene_==NULL){
	  std::cout << "Could not load mesh from file " << object_path << std::endl;
	  exit(-1);
	}

	if(scene_->mNumMeshes>1)
	  {
	    std::cout << "Object " << object_path << " consists of more than one mesh." << std::endl;
	    exit(-1);
	  }

	numFaces_ = scene_->mMeshes[0]->mNumFaces;
	numVertices_ = scene_->mMeshes[0]->mNumVertices;
	
	std::cout << "adding " << scene_->mNumMeshes 
		  << " meshes with " << numFaces_ 
		  << " faces and " << numVertices_ 
		  << " vertices" << std::endl;
	vertices_.resize(4,numVertices_);
	for(unsigned v=0;v<numVertices_;++v)
	    {
	      vertices_(0,v) = scene_->mMeshes[0]->mVertices[v].x;
	      vertices_(1,v) = scene_->mMeshes[0]->mVertices[v].y;
	      vertices_(2,v) = scene_->mMeshes[0]->mVertices[v].z;
	      vertices_(3,v) = 1;
	    }
	
	original_transform_ = Eigen::Affine3d::Identity();
      }
   
    void deallocateScene(){aiReleaseImport(scene_);}

    unsigned getNumFaces(){return numFaces_;}

    // given a new object transformation, update the original transform (an identity transform)
    void updateTransformation(const Eigen::Affine3d &p_tf){transform_ = p_tf * original_transform_;}

    // exchange the vertices in the search tree with the newly transformed ones
    void uploadVertices(TreeAndTri* search)
    {
      Eigen::MatrixXd trans_vertices = transform_.matrix() * vertices_;

      search->points.resize(numVertices_);
      for(unsigned v=0;v<numVertices_;++v)
	search->points[v] = K::Point_3(trans_vertices(0,v),
				       trans_vertices(1,v),
				       trans_vertices(2,v));
      return;
    }

    // Upload the triangle indices to the search tree
    // this would only need to be done ones in the beginning
    void uploadIndices(TreeAndTri* search)
    {
      search->triangles.resize(numFaces_);
      const struct aiMesh* mesh = scene_->mMeshes[0];
      for (unsigned f=0; f < numFaces_;++f)
	{
	  const struct aiFace* face_ai = &mesh->mFaces[f];
	  if(face_ai->mNumIndices!=3) {
	    std::cerr << "not a triangle!: " << face_ai->mNumIndices << " vertices" << std::endl;
	    throw;
	  }
	  
	  // faces contain the original vert indices; they should be offset by the verts
	  // of the previously processed meshes when one part can have multiple meshes!
	  search->triangles[f] = K::Triangle_3(search->points[face_ai->mIndices[0]], 
					       search->points[face_ai->mIndices[1]],
					       search->points[face_ai->mIndices[2]]);  
	}
      
      return;
    }

    // Upload the corresponding label to the tree which is associated to the face
    void uploadPartIDs(TreeAndTri* search, int id)
    {
      search->part_ids.resize(numFaces_);
      for (unsigned t = 0; t < numFaces_; ++t)
	search->part_ids[t] = id;
    }

  private:
    const struct aiScene* scene_;
    Eigen::MatrixXd vertices_;
    unsigned numFaces_;
    unsigned numVertices_;
    Eigen::Affine3d original_transform_;
    Eigen::Affine3d transform_;

  };
}//namespace render_kinect

#endif // _OBJECT_MESH_MODEL_
