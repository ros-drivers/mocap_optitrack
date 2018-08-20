#ifndef __MOCAP_OPTITRACK_RIGID_BODY_PUBLISHER_H__
#define __MOCAP_OPTITRACK_RIGID_BODY_PUBLISHER_H__

#include <map>
#include <memory>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <mocap_optitrack/version.h>
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>

namespace mocap_optitrack
{

/// \brief Encapsulation of a RigidBody data publisher.
class RigidBodyPublisher
{
public:
  RigidBodyPublisher(ros::NodeHandle &nh, 
    Version const& natNetVersion, 
    PublisherConfiguration const& config);
  ~RigidBodyPublisher();
  void publish(ros::Time const& time, RigidBody const&);

private:
  PublisherConfiguration config;

  bool useNewCoordinates;

  tf::TransformBroadcaster tfPublisher;
  ros::Publisher posePublisher;
  ros::Publisher pose2dPublisher;
};

/// \brief Dispatches RigidBody data to the correct publisher.
class RigidBodyPublishDispatcher
{
    typedef std::shared_ptr<RigidBodyPublisher> RigidBodyPublisherPtr;
    typedef std::map<int,RigidBodyPublisherPtr> RigidBodyPublisherMap;
    RigidBodyPublisherMap rigidBodyPublisherMap;

public:
    RigidBodyPublishDispatcher(ros::NodeHandle &nh, 
        Version const& natNetVersion, 
        PublisherConfigurations const& configs);
    void publish(ros::Time const& time, std::vector<RigidBody> const&);
};

} // namespace

#endif