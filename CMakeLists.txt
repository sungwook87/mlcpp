cmake_minimum_required(VERSION 2.8.3)
project(bridge_inspection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_geometry
#  message_generation
)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs image_geometry #message_runtime
  LIBRARIES
  OpenMP
)



include_directories(include ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )


add_executable(viewpoint_calc Viewpoint_calc.cpp include/stdafx.cpp)
add_dependencies(viewpoint_calc ${catkin_EXPORTED_TARGETS})
target_compile_options(viewpoint_calc PRIVATE ${OpenMP_FLAGS})
target_link_libraries(viewpoint_calc ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenMP_LIBS}
    pcl_common
    pcl_visualization
    pcl_registration
    pcl_filters
    pcl_sample_consensus
    pcl_octree
    pcl_io
    )

add_executable(viewpoint_calc_slice Viewpoint_calc_slice.cpp include/stdafx.cpp)
add_dependencies(viewpoint_calc_slice ${catkin_EXPORTED_TARGETS})
target_compile_options(viewpoint_calc_slice PRIVATE ${OpenMP_FLAGS})
target_link_libraries(viewpoint_calc_slice ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenMP_LIBS}
    pcl_common
    pcl_visualization
    pcl_registration
    pcl_filters
    pcl_sample_consensus
    pcl_octree
    pcl_io
    )
