#include "mocap_optitrack/data_model.h"

namespace mocap_optitrack
{

RigidBody::RigidBody()
{
}

const geometry_msgs::PoseStamped RigidBody::getRosPose(bool newCoordinates)
{
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.header.stamp = ros::Time::now();
  if (newCoordinates)
  {
    // Motive 1.7+ coordinate system
    ros_pose.pose.position.x = -pose.position.x;
    ros_pose.pose.position.y = pose.position.z;
    ros_pose.pose.position.z = pose.position.y;

    ros_pose.pose.orientation.x = -pose.orientation.x;
    ros_pose.pose.orientation.y = pose.orientation.z;
    ros_pose.pose.orientation.z = pose.orientation.y;
    ros_pose.pose.orientation.w = pose.orientation.w;
  }
  else
  {
    // y & z axes are swapped in the Optitrack coordinate system
    ros_pose.pose.position.x = pose.position.x;
    ros_pose.pose.position.y = -pose.position.z;
    ros_pose.pose.position.z = pose.position.y;

    ros_pose.pose.orientation.x = pose.orientation.x;
    ros_pose.pose.orientation.y = -pose.orientation.z;
    ros_pose.pose.orientation.z = pose.orientation.y;
    ros_pose.pose.orientation.w = pose.orientation.w;
  }
  return ros_pose;
}

bool RigidBody::hasData()
{
    static const char zero[sizeof(pose)] = { 0 };
    return memcmp(zero, (char*) &pose, sizeof(pose));
}

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

MarkerSet::MarkerSet() : 
    numMarkers(0), 
    markers(0) 
{}

ModelFrame::ModelFrame() : 
    latency(0.0)
{
}

ServerInfo::ServerInfo() :
    natNetVersion(0,0,0,0),
    serverVersion(0,0,0,0)
{

}

}