cmake_minimum_required(VERSION 2.8.3)
project(mocap_optitrack)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
