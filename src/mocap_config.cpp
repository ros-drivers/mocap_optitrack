
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "mocap_config.h"

const std::string POSE_TOPIC_PARAM_NAME = "pose";
const std::string POSE2D_TOPIC_PARAM_NAME = "pose2d";
const std::string TF_TOPIC_PARAM_NAME = "tf";

PublishedRigidBody::PublishedRigidBody(XmlRpc::XmlRpcValue &config_node)
{
  publish_pose = validateParam(config_node, POSE_TOPIC_PARAM_NAME);
  publish_pose2d = validateParam(config_node, POSE2D_TOPIC_PARAM_NAME);
  publish_tf = validateParam(config_node, TF_TOPIC_PARAM_NAME);

  if (publish_pose)
  {
    pose_topic = (std::string&) config_node[POSE_TOPIC_PARAM_NAME];
    pose_pub = n.advertise<geometry_msgs::Pose>(pose_topic, 1000);
  }

  if (publish_pose2d)
  {
    pose2d_topic = (std::string&) config_node[POSE2D_TOPIC_PARAM_NAME];
    pose2d_pub = n.advertise<geometry_msgs::Pose2D>(pose2d_topic, 1000);
  }

  if (publish_tf)
  {
    tf_topic = (std::string&) config_node[TF_TOPIC_PARAM_NAME];
  }
}

void PublishedRigidBody::publish(RigidBody &body)
{
  if (publish_pose)
    pose_pub.publish(body.pose);

  if (!publish_pose2d && !publish_tf)
    return;

  tf::Quaternion q(body.pose.orientation.x,
                   body.pose.orientation.z,
                   body.pose.orientation.y,
                   body.pose.orientation.w);

  double roll, pitch, yaw;
  btMatrix3x3(q).getEulerYPR(yaw, pitch, roll);

  // publish 2D pose
  if (publish_pose2d)
  {
    geometry_msgs::Pose2D pose;
    pose.x = body.pose.position.x;
    pose.y = body.pose.position.z;
    pose.theta = yaw;
    pose2d_pub.publish(pose);
  }

  if (publish_tf)
  {
    // publish transform
    tf::Transform transform;
    // Translate mocap data from mm --> m to be compatible with rviz
    transform.setOrigin( tf::Vector3(body.pose.position.x / 1000.0f,
                                     body.pose.position.y / 1000.0f,
                                     body.pose.position.z / 1000.0f));

    // Handle different coordinate systems (Arena vs. rviz)
    transform.setRotation(q.inverse());
    ros::Time timestamp(ros::Time::now());
    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, "base_link", std::string(tf_topic)));
  }
}

bool PublishedRigidBody::validateParam(XmlRpc::XmlRpcValue &config_node, const std::string &name)
{
  if (!config_node.hasMember(name))
  {
    return false;
  }

  if (config_node[name].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    return false;
  }

  return true;
}

