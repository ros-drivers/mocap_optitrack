#include "mocap_optitrack/mocap_calibrate.h"



namespace mocap_optitrack {

bool MocapCalibrate::calibrate(Calibration::Request& req,
                               Calibration::Response& res) {
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
  tf::StampedTransform mocap_to_map_tf(
      base_link_mocap.inverse() * base_link,
      base_link_mocap.stamp_, mocap_frame_, map_frame_);

  publish_mocap_to_map_tf(mocap_to_map_tf);
  return true;
}

void MocapCalibrate::publish_mocap_to_map_tf(
    tf::StampedTransform& transform_stamped) {
  std::cout << "MOCAP to MAP transform has been published\n";
  geometry_msgs::TransformStamped mocap_to_map_tf_msg;
  tf::transformStampedTFToMsg(transform_stamped, mocap_to_map_tf_msg);
  static_broadcaster_.sendTransform(mocap_to_map_tf_msg);
}

}
