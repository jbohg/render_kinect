cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set the path for project includes
include_directories(${PROJECT_SOURCE_DIR}/include)

# There exist different versions of the assimp library for different 
# ubuntu distros. 
execute_process(COMMAND lsb_release -sc 
  OUTPUT_VARIABLE _distro OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro}")
message("Compiling for Ubuntu version ${_distro}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message("Found OpenMP")
  include_directories(${OpenMP_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
endif(OPENMP_FOUND)

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

find_package(OpenCV QUIET REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  message("Found OpenCV")
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(OpenCV_LIBS ${OpenCV_LIBS} opencv_imgcodecs)
  set(LIBS ${LIBS} ${OpenCV_LIBS})
endif(OpenCV_FOUND)


add_library(${PROJECT_NAME} src/kinectSimulator.cpp src/noiseutils.cpp)
target_link_libraries(${PROJECT_NAME} assimp CGAL noise ${LIBS}) 

add_executable(render_object src/main_kinect.cpp)
target_link_libraries(render_object ${PROJECT_NAME})
