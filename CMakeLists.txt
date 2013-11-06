cmake_minimum_required(VERSION 2.8.3)
project(mocap_optitrack)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf)


catkin_package(
   INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)


SET( MOCAP_SRCS
  src/mocap_datapackets.cpp
  src/mocap_config.cpp
  src/Socket.cpp
  src/mocap_node.cpp
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mocap_node src/mocap_node.cpp src/mocap_config.cpp   src/mocap_datapackets.cpp   src/Socket.cpp)


target_link_libraries(mocap_node ${catkin_LIBRARIES})

 install(TARGETS 
   mocap_node mocap_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

