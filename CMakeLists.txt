cmake_minimum_required(VERSION 3.0.2)
project(shadowcalib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  fiducial_msgs
  tf2
  tf2_ros
  message_generation
)

# ========================== Find Eigen ========================================================= #
find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
	message(STATUS "Eigen3 found")
else ()
	message(FATAL_ERROR "Eigen3 not found")
endif ()

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        fiducial_msgs
        tf2
        tf2_ros
)

add_executable(aruco_tf src/aruco_tf.cpp)
target_link_libraries(aruco_tf ${catkin_LIBRARIES})