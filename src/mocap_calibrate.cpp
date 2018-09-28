#include "mocap_optitrack/mocap_calibrate.h"
#include <fstream>


namespace mocap_optitrack {

bool MocapCalibrate::calibrate(Calibration::Request& req, Calibration::Response& res) {
  std::cout << "MocapCalibrate called\n";
  tf::StampedTransform base_link;
  try {
    listener.lookupTransform(
        base_link_frame_, map_frame_, ros::Time(0), base_link);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }

  tf::StampedTransform base_link_mocap;
  try {
    listener.lookupTransform(
        base_link_mocap_frame_, mocap_frame_, ros::Time(0), base_link_mocap);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  // Calculate transform
  mocap_to_map_tf_ = tf::StampedTransform(base_link_mocap.inverse() * base_link,
                                          base_link_mocap.stamp_, mocap_frame_, map_frame_);
  publish_mocap_to_map_tf(mocap_to_map_tf_);
  return true;
}

bool MocapCalibrate::SaveCalibration(Calibration::Request& req, Calibration::Response& res) {
  std:ofstream file(calibration_data_path_, std::ios::in | std::ios::trunc);
  file << mocap_to_map_tf_.getOrigin().getX() << std::endl
      << mocap_to_map_tf_.getOrigin().getY() << std::endl
      << mocap_to_map_tf_.getOrigin().getZ() << std::endl
      << mocap_to_map_tf_.getRotation().getX() << std::endl
      << mocap_to_map_tf_.getRotation().getY() << std::endl
      << mocap_to_map_tf_.getRotation().getZ() << std::endl
      << mocap_to_map_tf_.getRotation().getW() << std::endl;
  std::cout << "Calibration data has been saved in file: " << calibration_data_path_ << "\n";
}

bool MocapCalibrate::LoadCalibration(Calibration::Request& req, Calibration::Response& res) {
  tf::Vector3 trans;
  tf::Quaternion rot;
  std::ifstream file(calibrtion_data_path_, std::ios::out);
  double val;
  file >> val; trans.setX(val);
  file >> val; trans.setY(val);
  file >> val; trans.setZ(val);

  file >> val; rot.setX(val);
  file >> val; rot.setY(val);
  file >> val; rot.setZ(val);
  file >> val; rot.setW(val);

  tf::Transform transform(rot, trans);
  mocap_to_map_tf_.setData(transform);
  publish_mocap_to_map_tf(mocap_to_map_tf_);
  std::cout << "Calibration data has been load from file: " << calibration_data_path_ << "\n";
}

void MocapCalibrate::publish_mocap_to_map_tf(
    tf::StampedTransform& transform_stamped) {
  std::cout << "MOCAP to MAP transform has been published\n";
  geometry_msgs::TransformStamped mocap_to_map_tf_msg;
  tf::transformStampedTFToMsg(transform_stamped, mocap_to_map_tf_msg);
  static_broadcaster_.sendTransform(mocap_to_map_tf_msg);
}

}
