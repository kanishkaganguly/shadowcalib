#include "../include/aruco_tf.hpp"

/**
 * @brief Save calibration data to file
 */
void ArucoTF::saveCalibToFile(const Eigen::Quaternionf &save_rot,
                              const Eigen::Vector3f &save_trans) {
  if (!ArucoTF::calib) {
    ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                        << "Saving calibration to file");
    std::string calib_path = ros::package::getPath("shadowcalib");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    std::vector<float> rot, trans;

    // Convert quaternion (w,x,y,z) from Eigen to Vector
    rot.push_back(save_rot.w());
    rot.push_back(save_rot.x());
    rot.push_back(save_rot.y());
    rot.push_back(save_rot.z());

    // Convert translation from Eigen to Vector
    trans.push_back(save_trans(0));
    trans.push_back(save_trans(1));
    trans.push_back(save_trans(2));

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json object
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      ROS_WARN_STREAM("JSON Parse failed: " << e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section =
        (ArucoTF::cam_prefix).substr(0, ArucoTF::cam_prefix.length() - 1);
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
    }

    // Add rotation and translation to json object
    calib_data[calib_section]["rot"] = rot;
    calib_data[calib_section]["trans"] = trans;

    std::ofstream calib_file_out(calib_path,
                                 std::fstream::out | std::ofstream::trunc);
    calib_file_out << std::setprecision(16) << calib_data;
    calib_file_out.close();
  }
}

/**
 * @brief Load calibration data from file
 */
void ArucoTF::loadCalibFromFile() {
  if (!ArucoTF::calib) {
    ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                        << "Loading calibration from file");
    std::string calib_path = ros::package::getPath("shadowcalib");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json object
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      ROS_WARN_STREAM("JSON Parse failed: " << e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section =
        (ArucoTF::cam_prefix).substr(0, ArucoTF::cam_prefix.length() - 1);
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
      // Get translation and rotation data from json
      std::vector<float> trans_json = calib_data[calib_section]["trans"];
      std::vector<float> rot_json = calib_data[calib_section]["rot"];

      // Vector in (x, y, z) format
      Eigen::Vector3f trans;
      trans(0) = trans_json[0];
      trans(1) = trans_json[1];
      trans(2) = trans_json[2];
      // Quaternion in (w,x,y,z) format
      Eigen::Quaternionf quat;
      quat.w() = rot_json[0];
      quat.x() = rot_json[1];
      quat.y() = rot_json[2];
      quat.z() = rot_json[3];

      std::cout << trans_json << std::endl;
      std::cout << rot_json << std::endl;

      // Apply calibration data to camera
      ArucoTF::setTFCamToWorld(quat, trans);
      // Set calibrated
      ArucoTF::calib = true;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
      ArucoTF::calib = false;
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
  ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                      << "Calibrating camera to world");
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

  // Set calibrated
  ArucoTF::calib = true;
}

/**
 * @brief Take samples of camera to marker and marker to world poses from robot
 */
void ArucoTF::takeCalibrationSamples() {
  if (ArucoTF::calib) {
    ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                        << "Already calibrated, exiting.");
    return;
  };

  int sample_cnt = 0;
  ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                      << "Move robot to pose...");
  ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                      << "Press ENTER to record sample.");

  while (sample_cnt < num_samples) {
    ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                        << "Pose: " << sample_cnt + 1 << "/"
                        << ArucoTF::num_samples);
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
  }
  ROS_INFO_ONCE("Calibration samples gathered");
}

/**
 * @brief Callback function to get marker pose in camera coordinates
 * Sets class variable with returned value
 */
void ArucoTF::lookup_camToMarker() {
  ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                      << "Getting aruco transform");

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
 * Returns obtained transform
 */
geometry_msgs::Transform ArucoTF::lookup_camToMarker(const int &marker_id) {
  ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                      << "Getting aruco transform for Marker " << marker_id);

  fiducial_msgs::FiducialTransformArray::ConstPtr fiducial_msg =
      ros::topic::waitForMessage<fiducial_msgs::FiducialTransformArray>(
          ArucoTF::aruco_transform_topic);

  for (fiducial_msgs::FiducialTransform marker : fiducial_msg->transforms) {
    if (marker.fiducial_id == marker_id) {
      geometry_msgs::Transform tform = marker.transform;
      return tform;
    }
  }

  throw(ArucoTF::NoTransformException());
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
      ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                          << "Getting rh_imu to world");
      ArucoTF::tform_markerToWorld =
          tfBuffer.lookupTransform("world", "rh_imu", ros::Time(0));
    } else {
      ArucoTF::tform_markerToWorld = geometry_msgs::TransformStamped();
      ROS_INFO_STREAM("[" << ArucoTF::cam_prefix.c_str() << "] "
                          << "Could not find transform from world to rh_imu");
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
  ROS_INFO_STREAM_THROTTLE(10, "[" << ArucoTF::cam_prefix.c_str() << "] "
                                   << "Broadcasting camera to world");
  ArucoTF::tform_camToWorld.header.stamp = ros::Time::now();
  ArucoTF::tform_camToWorld.header.frame_id = "world";
  ArucoTF::tform_camToWorld.child_frame_id = ArucoTF::cam_prefix + "cam";
  ArucoTF::tform_camToWorld.transform = tf2::toMsg(ArucoTF::tf_camToWorld);
  ArucoTF::br_camToWorld.sendTransform(ArucoTF::tform_camToWorld);
}

/**
 * @brief Broadcast marker pose from camera frame to world frame
 */
void ArucoTF::broadcast_allMarkersToWorld() {
  for (auto i : ArucoTF::aruco_track_targets) {
    ROS_INFO_STREAM_THROTTLE(10, "[" << ArucoTF::cam_prefix.c_str() << "] "
                                     << "Broadcasting marker_" << i
                                     << " to world");
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
    tform_newMarkerToWorld.child_frame_id =
        ArucoTF::cam_prefix + "marker_" + std::to_string(i);
    tform_newMarkerToWorld.transform = tf2::toMsg(tf_newMarkerToWorld);

    ArucoTF::br_markersToWorld.sendTransform(tform_newMarkerToWorld);
  }
}

/**
 * @brief Compute marker pose from camera frame to world frame
 */
void ArucoTF::lookup_allMarkersToWorld(const int &marker_id,
                                       tf2::Transform &tf_newMarkerToWorld) {
  try {
    // Get marker_id to cam
    geometry_msgs::Transform tform_camToNewMarker =
        ArucoTF::lookup_camToMarker(marker_id);
    tf2::Transform tf_camToNewMarker;
    tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

    // Transform to world frame
    tf_newMarkerToWorld = ArucoTF::tf_camToWorld * tf_camToNewMarker;
    tf_newMarkerToWorld.getRotation().normalize();

  } catch (const ArucoTF::NoTransformException &e) {
    ROS_WARN("%s", e.what());
    ros::Duration(1.0).sleep();
  }
}

/**
 * @brief Compare calibration marker to ideal transformation
 *
 * @param marker_id
 */
void ArucoTF::verifyCalibration(const int &marker_id) {
  // Get marker to world
  tf2::Transform tf_calibMarkerToWorld;
  ArucoTF::lookup_allMarkersToWorld(ArucoTF::aruco_calib_target,
                                    tf_calibMarkerToWorld);
  // Get rh_imu TF
  tf2::Stamped<tf2::Transform> tf_imuToWorld;
  ArucoTF::lookup_markerToWorld();
  tf2::fromMsg(ArucoTF::tform_markerToWorld, tf_imuToWorld);

  // Check errors between the two
  // float translation_error = ArucoTF::euclidean<float>(
  //     tf_calibMarkerToWorld.getOrigin(), tf_imuToWorld.getOrigin());
  // float quaternion_error =
  //     tf_calibMarkerToWorld.getRotation().dot(tf_imuToWorld.getRotation());
  // std::cout << "TranslationErr: " << translation_error
  //           << " RotationErr: " << quaternion_error << std::endl;
}

void broadcastMerged(ArucoTF *front_cam, ArucoTF *left_cam) {
  // Testing with static marker
  int marker_id = 5;
  // Camera view flag
  int flag = 0;
  // Empty transforms
  tf2::Transform tf_avgMarkerToWorld, tf_frontMarkerToWorld,
      tf_leftMarkerToWorld;

  // Convert back to geometry_msgs::TransformStamped
  geometry_msgs::TransformStamped tform_avgMarkerToWorld;
  tform_avgMarkerToWorld.header.stamp = ros::Time::now();
  tform_avgMarkerToWorld.header.frame_id = "world";
  tform_avgMarkerToWorld.child_frame_id =
      "avg_marker_" + std::to_string(marker_id);

  // Get marker_5 to world from front_cam
  try {
    ROS_INFO_STREAM_THROTTLE(10, "[FRONT] Looking up marker to world");
    front_cam->lookup_allMarkersToWorld(marker_id, tf_frontMarkerToWorld);
    flag += 1;
  } catch (ArucoTF::NoTransformException &e) {
    ROS_WARN("FRONT_CAM Lookup marker to world failed");
  }

  // Get marker_5 to world from left_cam
  try {
    ROS_INFO_STREAM_THROTTLE(10, "[LEFT] Looking up marker to world");
    left_cam->lookup_allMarkersToWorld(marker_id, tf_leftMarkerToWorld);
    flag += 2;
  } catch (ArucoTF::NoTransformException &e) {
    ROS_WARN("SIDE_CAM Lookup marker to world failed");
  }

  if (flag == 0) {
    ROS_INFO_STREAM_THROTTLE(10, "No markers found!");
    return;
  } else if (flag == 3) {
    // Both views found
    ROS_INFO_STREAM_THROTTLE(10, "[BOTH] Seen on both views");
    // Combine poses in tf2::Transform format
    ArucoTF::averagePoses(tf_frontMarkerToWorld, tf_leftMarkerToWorld,
                          tf_avgMarkerToWorld);
    tform_avgMarkerToWorld.transform = tf2::toMsg(tf_avgMarkerToWorld);
  } else if (flag == 1) {
    // Front view found
    ROS_INFO_STREAM_THROTTLE(10, "[FRONT] Seen on front view");
    tform_avgMarkerToWorld.transform = tf2::toMsg(tf_frontMarkerToWorld);
  } else if (flag == 2) {
    // Side view  found
    ROS_INFO_STREAM_THROTTLE(10, "[LEFT] Seen on left view");
    tform_avgMarkerToWorld.transform = tf2::toMsg(tf_leftMarkerToWorld);
  }
  front_cam->br_markersToWorld.sendTransform(tform_avgMarkerToWorld);
  ROS_INFO_STREAM_THROTTLE(
      10, "Broadcasting marker_" << std::to_string(marker_id) << " to world");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_tf");
  ros::NodeHandle n("~");
  ros::Rate rate(10.0);
  ROS_INFO("Started node");

  bool load_calib;
  bool verify_calib;
  int num_poses;
  // Load external calibration file
  n.param<bool>("load_calibration", load_calib, false);
  // Verify calibration
  n.param<bool>("verify_calibration", verify_calib, false);
  // Number of poses to use for calibration
  n.param<int>("num_poses", num_poses, 10);

  ROS_INFO("----------------------------------------------------------");
  // Set number of poses to capture for calibration
  ArucoTF calibrate_front(load_calib, num_poses, ArucoTF::CameraID::FRONT);
  // Front camera calibration
  if (!calibrate_front.load_calib) {
    calibrate_front.takeCalibrationSamples();
    calibrate_front.estimateTransformPointToPoint();
  } else {
    calibrate_front.loadCalibFromFile();
  }
  ROS_INFO("----------------------------------------------------------");
  ArucoTF calibrate_left(load_calib, num_poses, ArucoTF::CameraID::LEFT);
  // Left camera calibration
  if (!calibrate_left.load_calib) {
    calibrate_left.takeCalibrationSamples();
    calibrate_left.estimateTransformPointToPoint();
  } else {
    calibrate_left.loadCalibFromFile();
  }
  ROS_INFO("----------------------------------------------------------");
  // if (verify_calib) {
  //   calibrate_front.verifyCalibration(1);
  //   calibrate_left.verifyCalibration(1);
  // }

  while (n.ok()) {
    calibrate_front.broadcast_camToWorld();
    calibrate_left.broadcast_camToWorld();
    broadcastMerged(&calibrate_front, &calibrate_left);

    ros::spinOnce();
    rate.sleep();
  }
}