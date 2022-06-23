#pragma once
#include "prettyprint.hpp"
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <fstream>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
class ArucoTF {

public:
  ArucoTF(bool load_calib, int num_samples)
      : aruco_transform_topic("/fiducial_transforms"),
        ls_markerToWorld(tfBuffer), load_calib(load_calib),
        num_samples(num_samples), samples_camToMarker(3, num_samples),
        samples_markerToWorld(3, num_samples) {}

  template <class T>
  inline void rotate_quat(tf2::Transform &tf_orig,
                          const std::vector<T> &angle_rpy) {
    // Apply rotation
    tf2::Quaternion q_orig, q_rot;
    q_rot.setRPY(angles::from_degrees(angle_rpy[0]),
                 angles::from_degrees(angle_rpy[1]),
                 angles::from_degrees(angle_rpy[2]));
    q_orig = tf_orig.getRotation();
    q_orig *= q_rot;
    tf_orig.setRotation(q_orig);
  }

  // TF2 buffer
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener ls_markerToWorld;

  // Marker used for calibration
  const int aruco_calib_target = 1;
  // Markers used for tracking
  const std::vector<int> aruco_track_targets = {5, 7};
  // Check if calibration done
  bool calib = false;
  // Reuse existing calibration
  bool load_calib;

  // TransformMsg from camera to marker
  geometry_msgs::Transform tform_camToMarker;
  // TransformMsg from marker to world
  geometry_msgs::TransformStamped tform_markerToWorld;
  // Name of marker topic
  std::string aruco_transform_topic;
  // Get camera to marker transform
  void lookup_camToMarker();
  geometry_msgs::Transform lookup_camToMarker(const int &marker_id);
  // Get marker to world transform
  void lookup_markerToWorld();

  // TF Broadcaster
  tf2_ros::TransformBroadcaster br_camToWorld;
  tf2_ros::TransformBroadcaster br_markersToWorld;
  // Transform from camera to world
  tf2::Transform tf_camToWorld;
  // TransformMsg from camera to world
  geometry_msgs::TransformStamped tform_camToWorld;
  // Broadcast camera pose wrt world frame
  void broadcast_camToWorld();
  // Broadcast all other markers to world
  void broadcast_allMarkersToWorld();

  // 3D point to point transform estimation
  const int num_samples;
  void setTFCamToWorld(tf2::Quaternion &quat, tf2::Vector3 &trans);
  void setTFCamToWorld(Eigen::Quaternionf &quat, Eigen::Vector3f &trans);
  void estimateTransformPointToPoint();
  void takeCalibrationSamples();
  void saveCalibToFile(const Eigen::Quaternionf &save_rot,
                       const Eigen::Vector3f &save_trans);
  void loadCalibFromFile();
  Eigen::MatrixXf samples_camToMarker, samples_markerToWorld;
};