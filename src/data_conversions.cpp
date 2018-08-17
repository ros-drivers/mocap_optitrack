#include "data_conversions.h"

namespace mocap_optitrack
{
  geometry_msgs::PoseStamped getRosPose(RigidBody const& body, bool newCoordinates)
  {
    geometry_msgs::PoseStamped ros_pose;
    ros_pose.header.stamp = ros::Time::now();
    if (newCoordinates)
    {
      // Motive 1.7+ coordinate system
      ros_pose.pose.position.x = -body.pose.position.x;
      ros_pose.pose.position.y = body.pose.position.z;
      ros_pose.pose.position.z = body.pose.position.y;
  
      ros_pose.pose.orientation.x = -body.pose.orientation.x;
      ros_pose.pose.orientation.y = body.pose.orientation.z;
      ros_pose.pose.orientation.z = body.pose.orientation.y;
      ros_pose.pose.orientation.w = body.pose.orientation.w;
    }
    else
    {
      // y & z axes are swapped in the Optitrack coordinate system
      ros_pose.pose.position.x = body.pose.position.x;
      ros_pose.pose.position.y = -body.pose.position.z;
      ros_pose.pose.position.z = body.pose.position.y;
  
      ros_pose.pose.orientation.x = body.pose.orientation.x;
      ros_pose.pose.orientation.y = -body.pose.orientation.z;
      ros_pose.pose.orientation.z = body.pose.orientation.y;
      ros_pose.pose.orientation.w = body.pose.orientation.w;
    }
    return ros_pose;
  }   
}