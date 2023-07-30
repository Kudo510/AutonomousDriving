cmake_minimum_required(VERSION 3.0.2)
project(navigation_pkg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_pkg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/move_base_seq.py 
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)