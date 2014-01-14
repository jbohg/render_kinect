Simulation of Kinect Measurements
=============

This C++ project implements the Kinect sensor model as described in 

   BlenSor: Blender Sensor Simulation Toolbox Advances in Visual Computing. Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang. In Lecture Notes in Computer Science. pp 199--208. 2011. 

and as implemented in python as a Blender plugin. For more information, check out [the BlenSor Webpage](http://www.blensor.org)

This is a simplified version of this simulation accepting only a single rigid object.

Requirements
----------
The following libraries are required to compile the code:

* OpenCV (image I/O, filtering)
* CGAL (Fast Intersection Queries)
* Eigen (Linear Algebra)
* assimp (Mesh I/O)
* noise (Generation of Different Noise Types)

The following libraries are optional:

* OpenMP (Parallelization)
* PCL (Point cloud  I/O)

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
./render_object wheel.obj
```

This should store a number of depth and labeled images in /tmp. If you have PCL installed, it also stores point clouds as pcl files.

Point clouds generated from a simulated kinect measurement taken from a wheel in 10 different poses.
![](data/Wheels.png?raw=true)
