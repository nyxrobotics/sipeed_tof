cmake_minimum_required(VERSION 3.0.2)
project(sipeed_tof)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             roscpp
             rospy
             std_msgs
             message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES sipeed_tof_ms_a010
  CATKIN_DEPENDS
    message_runtime
    cv_bridge
    roscpp
    rospy
    std_msgs
  #  DEPENDS system_lib
)

include_directories(
  third_party/cJSON
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build sipeed_a010
add_executable(
  sipeed_a010
  src/sipeed_a010/sipeed_tof_ms_a010_node.cc
  src/sipeed_a010/frame_handle.cc
  third_party/cJSON/cJSON.c
)
target_link_libraries(
  sipeed_a010
  ${catkin_LIBRARIES}
  event
  jpeg
)
target_include_directories(sipeed_a010 PUBLIC include/sipeed_a010)
add_dependencies(
  sipeed_a010
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
install(
  TARGETS sipeed_a010
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build sipeed_a075
add_executable(
  sipeed_a075
  src/sipeed_a075/publisher.cpp
  src/sipeed_a075/process.cpp
  third_party/cJSON/cJSON.c
)
target_link_libraries(
  sipeed_a075
  ${catkin_LIBRARIES}
  event
  jpeg
)
target_include_directories(sipeed_a075 PUBLIC include/sipeed_a075)
add_dependencies(
  sipeed_a075
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
install(
  TARGETS sipeed_a075
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
