Simulation of Kinect Measurements
=============

This C++ project implements the Kinect sensor model as described in 

BlenSor: Blender Sensor Simulation Toolbox Advances in Visual Computing. Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang. In Lecture Notes in Computer Science. pp 199--208. 2011. 

and as implemented in python as a Blender plugin. For more information, check out [the BlenSor Webpage](http://www.blensor.org)

Requirements
----------
The following libraries are required to compile the code:
1. OpenCV
2. CGAL 
3. Eigen
4. assimp
5. noise

The following libraries are optional:
1. OpenMP (Parallelization)
2. PCL (Storing a point cloud as .pcd file)

Compilation
------------
```
mkdir build
cd build
cmake ..
make -j
```

Testing
------------
There is a small test program that will render the simulated kinect measurements of a wheel at a number of slightly perturbed transformations relative to the camera.

```
cd bin
render_object wheel.obj
```

This should store a number of depth and labeled images in /tmp. If you have PCL installed, it also stores point clouds as pcl files.