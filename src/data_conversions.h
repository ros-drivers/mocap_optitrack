#ifndef __MOCAP_OPTITRACK_DATA_CONVERSIONS_H__
#define __MOCAP_OPTITRACK_DATA_CONVERSIONS_H__

#include <geometry_msgs/PoseStamped.h>
#include <mocap_optitrack/data_model.h>

namespace mocap_optitrack
{
	geometry_msgs::PoseStamped getRosPose(RigidBody const& body, bool newCoordinates);
}


#endif