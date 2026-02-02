cmake_minimum_required(VERSION 3.10)
project(kinectSim)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
if(_distro)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro}")
  message("Compiling for Ubuntu version ${_distro}")
endif()

find_package(Eigen3 REQUIRED)
# Modern CMake uses Eigen3::Eigen target, but we also add include dir for compatibility
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
if(TARGET Eigen3::Eigen)
  set(EIGEN_TARGET Eigen3::Eigen)
endif()

find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message("Found OpenMP")
  include_directories(${OpenMP_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
endif(OPENMP_FOUND)

# PCL is optional (used only for point cloud I/O)
find_package(PCL 1.3 QUIET COMPONENTS io)
if(PCL_FOUND)
  message("Found PCL - enabling point cloud output")
  include_directories(${PCL_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${PCL_LIBRARIES})
  add_definitions(${PCL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_PCL")
else()
  message("PCL not found - point cloud output disabled")
endif()

find_package(OpenCV REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  message("Found OpenCV")
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(OpenCV_LIBS ${OpenCV_LIBS} opencv_imgcodecs)
  set(LIBS ${LIBS} ${OpenCV_LIBS})
endif(OpenCV_FOUND)

add_library(${PROJECT_NAME} src/kinectSimulator.cpp src/noiseutils.cpp)
target_link_libraries(${PROJECT_NAME} assimp CGAL noise ${LIBS})
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
endif()

add_executable(render_object src/main_kinect.cpp)
target_link_libraries(render_object ${PROJECT_NAME})
