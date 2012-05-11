

#ifndef __MOCAP_CONFIG_H__
#define __MOCAP_CONFIG_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "mocap_datapackets.h"

class PublishedRigidBody
{
  private:
  ros::NodeHandle n;

  std::string pose_topic;
  std::string pose2d_topic;
  std::string tf_topic;

  bool publish_pose;
  bool publish_tf;
  bool publish_pose2d;

  tf::TransformBroadcaster tf_pub;
  ros::Publisher pose_pub;
  ros::Publisher pose2d_pub;

  bool validateParam(XmlRpc::XmlRpcValue &, const std::string &);

  public:
  PublishedRigidBody(XmlRpc::XmlRpcValue &);
  void publish(RigidBody &);
};

typedef std::map<int, PublishedRigidBody> RigidBodyMap;
typedef std::pair<int, PublishedRigidBody> RigidBodyItem;

#endif // __MOCAP_CONFIG_H__
