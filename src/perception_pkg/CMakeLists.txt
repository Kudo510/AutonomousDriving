cmake_minimum_required(VERSION 3.0.2)
project(perception_pkg)

# Check for C++14 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

# Set C++ standard
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wl,--no-as-needed")
else()
    message(FATAL_ERROR "Unsupported compiler -- at least C++14 support is needed!")
endif()

set(octomap_DIR /opt/ros/noetic/share/octomap)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf_conversions
  sensor_msgs
  octomap
)

find_package(Eigen3)
find_package(PCL 1.10 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
#find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  tf
  eigen_conversions
  tf_conversions
  sensor_msgs
)

catkin_install_python(PROGRAMS scripts/odometry.py 
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_filtering src/point_cloud_filtering.cpp)
add_dependencies(point_cloud_filtering ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_filtering ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries(point_cloud_filtering ${PCL_COMMON_LIBRARIES}  ${PCL_LIBRARIES})

#add_executable(global_2dmap_postprocessing src/global_2dmap_postprocessing.cpp)
#add_dependencies(global_2dmap_postprocessing ${catkin_EXPORTED_TARGETS})
#target_link_libraries(global_2dmap_postprocessing ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(localization_node src/localization.cpp)
#add_dependencies(localization_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(localization_node ${catkin_LIBRARIES})