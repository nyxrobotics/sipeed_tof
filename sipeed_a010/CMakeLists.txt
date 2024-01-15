cmake_minimum_required(VERSION 3.0.2)
project(sipeed_a010)

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             roscpp
             rospy
             std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES sipeed_tof_ms_a010
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/sipeed_tof_ms_a010_node.cc
  src/cJSON.c
  src/frame_handle.cc
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(
  ${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
