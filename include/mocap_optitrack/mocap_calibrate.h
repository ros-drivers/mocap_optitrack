#pragma once

#include "ros/ros.h"
#include "mocap_optitrack/Calibration.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

namespace mocap_optitrack {

class MocapCalibrate {
public:
  bool calibrate(Calibration::Request& req,
                 Calibration::Response& res);

  void publish_mocap_to_map_tf(tf::StampedTransform& tf);

  std::string mocap_frame_ = "ground_truth";
  std::string base_link_mocap_frame_ = "ground_truth/base_link";
  std::string map_frame_ = "map";
  std::string base_link_frame_ = "base_link";
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf::TransformListener listener;
};

}
