#include "../include/aruco_tf.hpp"

/**
 * @brief Save calibration data to file
 */
void ArucoTF::saveCalibToFile(const Eigen::Quaternionf &save_rot,
                              const Eigen::Vector3f &save_trans) {
  ROS_INFO("Saving calibration to file");
  std::ofstream calib_file;
  std::string calib_path = ros::package::getPath("shadowteleop");
  calib_path += "/calibration/camera/ur10_calib.csv";
  calib_file.open(calib_path,
                  std::fstream::in | std::fstream::out | std::fstream::trunc);
  // T(x, y, z)R(w,x,y,z)
  calib_file << save_trans(0) << "," << save_trans(1) << "," << save_trans(2)
             << ",";
  calib_file << save_rot.w() << "," << save_rot.x() << "," << save_rot.y()
             << "," << save_rot.z();
  calib_file.close();
}

/**
 * @brief Load calibration data from file
 */
void ArucoTF::loadCalibFromFile() {
  if (!ArucoTF::calib) {
    ROS_INFO("Loading calibration from file");
    std::string calib_data;
    std::ifstream calib_file;
    std::string calib_path = ros::package::getPath("shadowteleop");
    calib_path += "/calibration/camera/ur10_calib.csv";

    calib_file.open(calib_path);
    if (calib_file.is_open()) {
      std::getline(calib_file, calib_data);

      std::vector<float> fcalib_data;
      std::stringstream ss(calib_data);
      while (ss.good()) {
        std::string csv;
        std::getline(ss, csv, ',');
        fcalib_data.push_back(std::stof(csv));
      }
      Eigen::Vector3f trans(fcalib_data[0], fcalib_data[1], fcalib_data[2]);
      Eigen::Quaternionf quat(fcalib_data[3], fcalib_data[4], fcalib_data[5],
                              fcalib_data[6]);
      quat.normalize();
      ArucoTF::setTFCamToWorld(quat, trans);

      calib_file.close();
      ArucoTF::calib = true;
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
  }
}

void ArucoTF::setTFCamToWorld(tf2::Quaternion &quat, tf2::Vector3 &trans) {
  // Convert to tf2
  ArucoTF::tf_camToWorld.setRotation(quat);
  ArucoTF::tf_camToWorld.setOrigin(trans);
}

void ArucoTF::setTFCamToWorld(Eigen::Quaternionf &quat,
                              Eigen::Vector3f &trans) {
  // Get rotation in tf2
  tf2::Quaternion rot_quat_camToWorld(quat.x(), quat.y(), quat.z(), quat.w());
  rot_quat_camToWorld.normalize();
  // Get translation in tf2
  tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

  // Convert to tf2
  ArucoTF::tf_camToWorld.setRotation(rot_quat_camToWorld);
  ArucoTF::tf_camToWorld.setOrigin(trans_camToWorld);
}

/**
 * @brief Get transform from source to destination given 3D point
 * correspondences.
 */

void ArucoTF::estimateTransformPointToPoint() {
  // Compute transform
  Eigen::Matrix4f tf_srcToDst = Eigen::Matrix4f::Zero();
  ROS_INFO("Calibrating camera to world");
  tf_srcToDst = Eigen::umeyama(ArucoTF::samples_camToMarker,
                               ArucoTF::samples_markerToWorld);

  // Get rotation
  Eigen::Matrix3f rot = tf_srcToDst.topLeftCorner(3, 3);
  Eigen::Quaternionf rot_quat(rot);
  tf2::Quaternion rot_quat_camToWorld(rot_quat.x(), rot_quat.y(), rot_quat.z(),
                                      rot_quat.w());

  // Get translation
  Eigen::Vector3f trans = tf_srcToDst.topRightCorner(3, 1);
  tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

  // Convert to tf2
  ArucoTF::setTFCamToWorld(rot_quat_camToWorld, trans_camToWorld);

  // Save data to file
  ArucoTF::saveCalibToFile(rot_quat, trans);
}

/**
 * @brief Take samples of camera to marker and marker to world poses from robot
 */
void ArucoTF::takeCalibrationSamples() {
  if (ArucoTF::calib) {
    return;
  };

  int sample_cnt = 0;
  std::cout << "Move robot to pose\n";
  std::cout << "Press ENTER to record sample." << std::endl;
  while (!ArucoTF::calib) {
    std::cout << "Pose: " << sample_cnt + 1 << "/" << ArucoTF::num_samples
              << std::endl;
    char c = getchar();

    ArucoTF::lookup_camToMarker();
    ArucoTF::samples_camToMarker.col(sample_cnt) =
        Eigen::Vector3f(ArucoTF::tform_camToMarker.translation.x,
                        ArucoTF::tform_camToMarker.translation.y,
                        ArucoTF::tform_camToMarker.translation.z)
            .transpose();

    ArucoTF::lookup_markerToWorld();
    ArucoTF::samples_markerToWorld.col(sample_cnt) =
        Eigen::Vector3f(ArucoTF::tform_markerToWorld.transform.translation.x,
                        ArucoTF::tform_markerToWorld.transform.translation.y,
                        ArucoTF::tform_markerToWorld.transform.translation.z)
            .transpose();

    sample_cnt++;
    if (sample_cnt == ArucoTF::num_samples) {
      ArucoTF::calib = true;
    }
  }
  ROS_INFO_ONCE("Calibration samples gathered");
}

/**
 * @brief Callback function to get marker pose in camera coordinates
 */
void ArucoTF::lookup_camToMarker() {
  ROS_INFO("Getting aruco transform");

  fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg =
      ros::topic::waitForMessage<fiducial_msgs::FiducialTransformArray>(
          ArucoTF::aruco_transform_topic);

  for (fiducial_msgs::FiducialTransform marker : fiducial_msg->transforms) {
    if (marker.fiducial_id == aruco_calib_target) {
      ArucoTF::tform_camToMarker = marker.transform;
    }
  }
}

/**
 * @brief Callback function to get marker pose in camera coordinates
 * Overloaded to accept marker ID
 */
geometry_msgs::Transform ArucoTF::lookup_camToMarker(const int &marker_id) {
  ROS_INFO_STREAM("Getting aruco transform for Marker " << marker_id);

  fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg =
      ros::topic::waitForMessage<fiducial_msgs::FiducialTransformArray>(
          ArucoTF::aruco_transform_topic);

  for (fiducial_msgs::FiducialTransform marker : fiducial_msg->transforms) {
    if (marker.fiducial_id == marker_id) {
      geometry_msgs::Transform tform = marker.transform;
      return tform;
    }
  }
}

/**
 * @brief Get transform from rh_imu to world frame
 * The marker is placed at the rh_imu location on the Hand
 * lookupTransform goes to target frame from source
 */
void ArucoTF::lookup_markerToWorld() {
  // TF2 listener for marker to world
  try {
    if (ArucoTF::tfBuffer.canTransform("world", "rh_imu", ros::Time(0))) {
      ROS_INFO("Getting rh_imu to world");
      ArucoTF::tform_markerToWorld =
          tfBuffer.lookupTransform("world", "rh_imu", ros::Time(0));
    } else {
      ArucoTF::tform_markerToWorld = geometry_msgs::TransformStamped();
      ROS_INFO("Could not find transform from world to rh_imu");
    }
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
 * @brief Broadcast camera pose with respect to world
 */
void ArucoTF::broadcast_camToWorld() {
  ROS_INFO_THROTTLE(10, "Broadcasting camera to world");
  ArucoTF::tform_camToWorld.header.stamp = ros::Time::now();
  ArucoTF::tform_camToWorld.header.frame_id = "world";
  ArucoTF::tform_camToWorld.child_frame_id = "cam";
  ArucoTF::tform_camToWorld.transform = tf2::toMsg(ArucoTF::tf_camToWorld);
  ArucoTF::br_camToWorld.sendTransform(ArucoTF::tform_camToWorld);
}

/**
 * @brief Broadcast marker pose from camera frame to world frame
 */
void ArucoTF::broadcast_allMarkersToWorld() {
  ROS_INFO_THROTTLE(10, "Broadcasting markers to world");

  for (auto i : ArucoTF::aruco_track_targets) {
    ROS_INFO_STREAM_THROTTLE(10, "Broadcasting marker_" << i << " to world");
    // Get marker_i to cam
    geometry_msgs::Transform tform_camToNewMarker =
        ArucoTF::lookup_camToMarker(i);
    tf2::Transform tf_camToNewMarker;
    tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

    // Transform to world frame
    tf2::Transform tf_newMarkerToWorld =
        ArucoTF::tf_camToWorld * tf_camToNewMarker;

    // Convert back to geometry_msgs::TransformStamped
    geometry_msgs::TransformStamped tform_newMarkerToWorld;
    tform_newMarkerToWorld.header.stamp = ros::Time::now();
    tform_newMarkerToWorld.header.frame_id = "world";
    tform_newMarkerToWorld.child_frame_id = "marker_" + std::to_string(i);
    tform_newMarkerToWorld.transform = tf2::toMsg(tf_newMarkerToWorld);

    ArucoTF::br_markersToWorld.sendTransform(tform_newMarkerToWorld);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_tf");
  ros::NodeHandle n("~");
  ros::Rate rate(10.0);
  ROS_INFO("Started node");

  bool load_calib;
  int num_poses;
  // Load external calibration file
  n.param<bool>("load_calibration", load_calib, false);
  // Number of poses to use for calibration
  n.param<int>("num_poses", num_poses, 10);

  // Set number of poses to capture for calibration
  ArucoTF left_calib(load_calib, num_poses);
  ArucoTF right_calib(load_calib, num_poses);

  ROS_INFO("Started aruco subscriber");

  while (n.ok()) {
    if (!calibrate.load_calib) {
      calibrate.takeCalibrationSamples();
      calibrate.estimateTransformPointToPoint();
    } else {
      calibrate.loadCalibFromFile();
    }
    calibrate.broadcast_camToWorld();
    calibrate.broadcast_allMarkersToWorld();
    ros::spinOnce();
    rate.sleep();
  }
}