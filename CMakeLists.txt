cmake_minimum_required(VERSION 2.8.3)
project(render_kinect)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  COMPONENTS
    roslib
    roscpp
    message_generation
    sensor_msgs
    visualization_msgs
    image_transport
    cv_bridge
    tf
    tf_conversions
    state_filtering
  )

# There exist different versions of the assimp library for different 
# ubuntu distros. 
execute_process(COMMAND lsb_release -sc 
  OUTPUT_VARIABLE _distro OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro} -frounding-math")
message("Compiling for Ubuntu version ${_distro}")

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(OpenCV QUIET REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  message("Found OpenCV")
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenCV_LIBS})
endif(OpenCV_FOUND)

find_package(PCL 1.3 QUIET REQUIRED COMPONENTS io)
if(PCL_FOUND)
  message("Found PCL")
  include_directories(${PCL_INCLUDE_DIRS})
  # in case your PCL installation is partt of ros you might need to add this include path
  #include_directories(${PCL_INCLUDE_DIRS} /opt/ros/<ros_distro>/include/)
  set(LIBS ${LIBS} ${PCL_LIBRARIES})
  add_definitions(${PCL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_PCL")
endif(PCL_FOUND)

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message("Found OpenMP")
  include_directories(${OpenMP_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
endif(OPENMP_FOUND)




## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  sensor_msgs  # Or other packages containing msgs
#  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roslib 
    message_runtime
    visualization_msgs
    image_transport 
    roscpp 
    cv_bridge
    tf
    tf_conversions
    state_filtering
  DEPENDS OpenCV eigen 
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE HEADERS include/*.hpp include/*.h)

add_library(${PROJECT_NAME} src/kinectSimulator.cpp src/noiseutils.cpp ${HEADERS})
target_link_libraries(${PROJECT_NAME} assimp CGAL noise ${LIBS} ${catkin_LIBRARIES})

add_executable(render_object src/main_kinect.cpp)
target_link_libraries(render_object ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(move_object src/move_object.cpp)
target_link_libraries(move_object ${PROJECT_NAME} yaml-cpp ${catkin_LIBRARIES})

add_executable(generate_dataset src/generate_dataset.cpp)
target_link_libraries(generate_dataset ${PROJECT_NAME} yaml-cpp ${catkin_LIBRARIES})
