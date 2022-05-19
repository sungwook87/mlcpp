cmake_minimum_required(VERSION 3.4)
project(mlcpp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_geometry
  pcl_ros
  pcl_conversions
  cv_bridge
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
  message(WARNING "OpenCV Found: " ${OpenCV_VERSION})
endif()
if(PCL_FOUND)
  message(WARNING "PCL Found: " ${PCL_VERSION})
endif()
if(OPENMP_FOUND)
  message(WARNING "OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_slice_node src/main_node.cpp)
target_link_libraries(${PROJECT_NAME}_slice_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OpenMP_LIBS}
  pcl_common
  pcl_registration
  pcl_filters
  pcl_sample_consensus
  pcl_octree
  pcl_io
)