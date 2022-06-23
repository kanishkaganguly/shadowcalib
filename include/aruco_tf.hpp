#pragma once
#include "prettyprint.hpp"
#include "json.hpp"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
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
#include <exception>

std::ostream &operator<<(std::ostream &os, const tf2::Vector3 &vec) {
  os << "tf2::Vector3:\n";
  os << "    x: " << vec.x() << "\n";
  os << "    y: " << vec.y() << "\n";
  os << "    z: " << vec.z() << "\n";
  return os;
}

std::ostream &operator<<(std::ostream &os, const tf2::Quaternion &quat) {
  os << "tf2::Quaternion:\n";
  os << "    w: " << quat.w() << "\n";
  os << "    x: " << quat.x() << "\n";
  os << "    y: " << quat.y() << "\n";
  os << "    z: " << quat.z() << "\n";
  return os;
}

template <class T>
std::ostream &operator<<(std::ostream &os, const Eigen::Quaternion<T> &quat) {
  os << "Eigen::Quaternion:\n";
  os << "    x: " << quat.x() << "\n";
  os << "    y: " << quat.y() << "\n";
  os << "    z: " << quat.z() << "\n";
  os << "    w: " << quat.w() << "\n";
  return os;
}

std::ostream &operator<<(std::ostream &os, const tf2::Transform &tf) {
  os << "tf2::transform:\n";
  os << "  translation:\n";
  os << "    x: " << tf.getOrigin().x() << "\n";
  os << "    y: " << tf.getOrigin().y() << "\n";
  os << "    z: " << tf.getOrigin().z() << "\n";
  os << "  rotation:\n";
  os << "    x: " << tf.getRotation().x() << "\n";
  os << "    y: " << tf.getRotation().y() << "\n";
  os << "    z: " << tf.getRotation().z() << "\n";
  os << "    w: " << tf.getRotation().w() << "\n";
  return os;
}

inline static void QuatConjugate(const tf2::Quaternion &in,
                                 tf2::Quaternion &conj) {
  conj.setX(-in.x());
  conj.setY(-in.y());
  conj.setZ(-in.z());
  conj.setW(in.w());
}

inline static void QuatConjugate(tf2::Quaternion &in) {
  in.setX(-in.x());
  in.setY(-in.y());
  in.setZ(-in.z());
}

class ArucoTF {
 public:
  enum CameraID { FRONT, LEFT };

  ArucoTF(bool load_calib, int num_samples, CameraID cam_id)
      : ls_markerToWorld(tfBuffer),
        load_calib(load_calib),
        num_samples(num_samples),
        samples_camToMarker(3, num_samples),
        samples_markerToWorld(3, num_samples),
        cam_id(cam_id) {
    if (cam_id == CameraID::FRONT) {
      cam_prefix = "front_";
      ROS_INFO("Calibrating FRONT CAMERA");
    } else if (cam_id == CameraID::LEFT) {
      cam_prefix = "left_";
      ROS_INFO("Calibrating SIDE CAMERA");
    }
    aruco_transform_topic =
        "/" + cam_prefix + "logitech_webcam/fiducial_transforms";
  }

  /**
   * @brief Custom exception when required transformation
   * is not found
   */
  struct NoTransformException : public std::exception {
    virtual const char *what() const throw() {
      return "Expected TF lookup not found";
    }
  };

  /**
   * @brief Euclidean distance between two tf2::Vector3 objects
   *
   */
  template <class T>
  inline T euclidean(tf2::Vector3 &vec1, tf2::Vector3 &vec2) {
    return std::sqrt(std::pow(2, (vec2[0] - vec1[0])) +
                     std::pow(2, (vec2[1] - vec1[1])) +
                     std::pow(2, (vec2[2] - vec1[2])));
  }

  /**
   * @brief Rotate tf2/Transform quaternion component by some Roll-Pitch-Yaw
   * angle
   */
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

  /**
   * @brief Convert geometry_msgs/Transform type to Eigen quaternion/vector
   * pairs
   */
  inline static void geometryMsgToEigen(const geometry_msgs::Transform &geo_msg,
                                        Eigen::Quaternionf &quat,
                                        Eigen::Vector3f &trans) {
    quat.x() = geo_msg.rotation.x;
    quat.y() = geo_msg.rotation.y;
    quat.z() = geo_msg.rotation.z;
    quat.w() = geo_msg.rotation.w;

    trans << geo_msg.translation.x, geo_msg.translation.y,
        geo_msg.translation.z;
  }

  /**
   * @brief Convert tf2/Transform type to Eigen quaternion/vector
   * pairs
   */
  inline static void tf2TransformToEigen(const tf2::Transform &tf2_msg,
                                         Eigen::Quaternionf &quat,
                                         Eigen::Vector3f &trans) {
    quat.x() = tf2_msg.getRotation().x();
    quat.y() = tf2_msg.getRotation().y();
    quat.z() = tf2_msg.getRotation().z();
    quat.w() = tf2_msg.getRotation().w();

    trans << tf2_msg.getOrigin()[0], tf2_msg.getOrigin()[1],
        tf2_msg.getOrigin()[2];
  }

  /**
   * @brief Merge poses of the same marker from multiple camera views
   * by weighted average into a single combined pose
   */
  inline static void averagePoses(const Eigen::Quaternionf &quat1,
                                  const Eigen::Vector3f &trans1,
                                  const Eigen::Quaternionf &quat2,
                                  const Eigen::Vector3f &trans2,
                                  Eigen::Quaternionf &avg_quat,
                                  Eigen::Vector3f &avg_trans) {
    avg_quat = quat2.slerp(0.5, quat1);
    avg_trans = (trans1 + trans2) / 2;
  }

  /**
   * @brief Averaging function for Quaternions using Eigenvalues method
   * @see
   * https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
   * @see
   * https://www.mathworks.com/matlabcentral/fileexchange/40098-tolgabirdal-averaging_quaternions

  # Q is a Nx4 numpy matrix and contains the quaternions to average in the
  rows. The quaternions are arranged as (w,x,y,z), with w being the scalar.
  The result will be the average quaternion of the input. Note that the signs of
  the output quaternion can be reversed, since q and -q describe the same
  orientation

  def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = numpy.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)
   */

  inline static void averageQuaternions(const tf2::Quaternion &quat1,
                                        const tf2::Quaternion &quat2,
                                        tf2::Quaternion &avg_quat) {
    Eigen::Vector4f eigQuat1, eigQuat2;
    if (quat1.w() < 0) {
      eigQuat1 << -quat1.w(), -quat1.x(), -quat1.y(), -quat1.z();
    } else {
      eigQuat1 << quat1.w(), quat1.x(), quat1.y(), quat1.z();
    }

    if (quat2.w() < 0) {
      eigQuat2 << -quat2.w(), -quat2.x(), -quat2.y(), -quat2.z();
    } else {
      eigQuat2 << quat2.w(), quat2.x(), quat2.y(), quat2.z();
    }

    // Number of quaternions to average
    int M = 2;
    // Size of quaternion (i.e. 4)
    int N = 4;
    // k largest eigenvectors
    int k = 1;
    // Placeholder matrix
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    // Multiply q with its transposed version q' and add A
    A = (eigQuat1 * eigQuat1.transpose()) + A;
    A = (eigQuat2 * eigQuat2.transpose()) + A;
    // Scale
    A = (1.0 / M) * A;
    // Compute eigenvalues and -vectors eigenValues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> eigensolver(N);
    eigensolver.compute(A);
    // Sort by largest
    Eigen::VectorXcf eigenVals = eigensolver.eigenvalues().reverse();
    Eigen::MatrixXcf eigenVecs =
        eigensolver.eigenvectors().block(0, N - k, N, k).rowwise().reverse();
    avg_quat.setW(eigenVecs.real().coeff(0));
    avg_quat.setX(eigenVecs.real().coeff(1));
    avg_quat.setY(eigenVecs.real().coeff(2));
    avg_quat.setZ(eigenVecs.real().coeff(3));
  }

  /**
   * @brief Merge poses of the same marker from multiple camera views
   * by weighted average into a single combined pose
   */
  inline static void averagePoses(const tf2::Transform &tf1,
                                  const tf2::Transform &tf2,
                                  tf2::Transform &avg_tf) {
    // Testing eigenvalue decomposition method
    tf2::Quaternion avg_quat;
    averageQuaternions(tf1.getRotation(), tf2.getRotation(), avg_quat);

    tf2::Quaternion identity_quat = tf2::Quaternion::getIdentity();
    // Make averaged quaternion zero angle with plane
    tf2::Quaternion avg_conj, angle_delta;
    QuatConjugate(avg_quat, avg_conj);
    angle_delta = avg_conj * identity_quat;
    avg_quat *= angle_delta;

    tf2::Vector3 avg_trans;
    avg_trans.setX((tf1.getOrigin().x() + tf2.getOrigin().x()) / 2);
    avg_trans.setY((tf1.getOrigin().y() + tf2.getOrigin().y()) / 2);
    avg_trans.setZ((tf1.getOrigin().z() + tf2.getOrigin().z()) / 2);
    avg_tf.setOrigin(avg_trans);
    avg_tf.setRotation(avg_quat);
  }

  // TF2 buffer
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener ls_markerToWorld;

  // Camera ID
  CameraID cam_id;
  // Camera Prefix
  std::string cam_prefix;
  // Marker used for calibration
  const int aruco_calib_target = 1;
  // Markers used for tracking
  const std::vector<int> aruco_track_targets = {5};
  // Check if calibration done
  bool calib = false;
  // Reuse existing calibration
  bool load_calib;
  // Number of calibration samples
  const int num_samples;

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
  // Lookup any markers to world
  void lookup_allMarkersToWorld(const int &marker_id,
                                tf2::Transform &tf_newMarkerToWorld);

  // 3D point to point transform estimation
  void setTFCamToWorld(tf2::Quaternion &quat, tf2::Vector3 &trans);
  void setTFCamToWorld(Eigen::Quaternionf &quat, Eigen::Vector3f &trans);
  void estimateTransformPointToPoint();
  void takeCalibrationSamples();
  void saveCalibToFile(const Eigen::Quaternionf &save_rot,
                       const Eigen::Vector3f &save_trans);
  void loadCalibFromFile();
  void verifyCalibration(const int &marker_id);
  Eigen::MatrixXf samples_camToMarker, samples_markerToWorld;
};