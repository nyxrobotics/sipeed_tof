cmake_minimum_required(VERSION 3.0.2)
project(sipeed_a075)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             roscpp
             rospy
             std_msgs
             message_generation
)
find_package(OpenCV REQUIRED)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES sipeed_tof_cpp
  CATKIN_DEPENDS
    message_runtime
    cv_bridge
    roscpp
    rospy
    std_msgs
  #  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
  publisher
  src/publisher.cpp
  src/process.cpp
  src/cJSON.c
)
target_link_libraries(
  publisher
  event
  jpeg
)

add_dependencies(
  publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  publisher
  ${catkin_LIBRARIES}
  event
  jpeg
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
