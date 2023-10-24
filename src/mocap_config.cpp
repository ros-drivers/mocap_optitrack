/* 
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <string>

#include "mocap_optitrack/mocap_config.h"

namespace mocap_optitrack
{
// Server description defaults
const int ServerDescription::Default::CommandPort = 1511;
const int ServerDescription::Default::DataPort   = 9001;
const std::string ServerDescription::Default::MulticastIpAddress = "224.0.0.251";
const bool ServerDescription::Default::EnableOptitrack = true;

// Param keys
namespace rosparam
{
  namespace keys
  {
    const std::string MulticastIpAddress = "optitrack_config.multicast_address";
    const std::string CommandPort = "optitrack_config.command_port";
    const std::string DataPort = "optitrack_config.data_port";
    const std::string EnableOptitrack = "optitrack_config/enable_optitrack";
    const std::string Version = "optitrack_config.version";
    const std::string RigidBodies = "rigid_bodies";
    const std::string PoseTopicName = "pose";
    const std::string Pose2dTopicName = "pose2d";
    const std::string OdomTopicName = "odom";
    const std::string EnableTfPublisher = "tf";
    const std::string ChildFrameId = "child_frame_id";
    const std::string ParentFrameId = "parent_frame_id";
  }
}

ServerDescription::ServerDescription() :
  commandPort(ServerDescription::Default::CommandPort),
  dataPort(ServerDescription::Default::DataPort),
  multicastIpAddress(ServerDescription::Default::MulticastIpAddress)
{}

void NodeConfiguration::fromRosParam(
  rclcpp::Node::SharedPtr& node, 
  ServerDescription& serverDescription, 
  PublisherConfigurations& pubConfigs)
{
  // Get server cconfiguration from ROS parameter server
  
  if(!node->get_parameter_or(
    rosparam::keys::MulticastIpAddress, 
    serverDescription.multicastIpAddress, 
    ServerDescription::Default::MulticastIpAddress)
  ) {
    RCLCPP_WARN(node->get_logger(), 
      "Could not get multicast address, using default: %s", serverDescription.multicastIpAddress.c_str());
  }

  if(!node->get_parameter_or(
    rosparam::keys::CommandPort, 
    serverDescription.commandPort, 
    ServerDescription::Default::CommandPort)
  ) {
    RCLCPP_WARN(node->get_logger(), 
      "Could not get command port, using default: %i", serverDescription.commandPort);
  }

  if(!node->get_parameter_or(
    rosparam::keys::EnableOptitrack,
    serverDescription.enableOptitrack,
    ServerDescription::Default::EnableOptitrack)
  ) {
    RCLCPP_WARN(node->get_logger(),
                "Could not get enable optitrack, using default: %d", serverDescription.enableOptitrack);
  }

  if(!node->get_parameter_or(
    rosparam::keys::DataPort, 
    serverDescription.dataPort, 
    ServerDescription::Default::DataPort)
  ) {
    RCLCPP_WARN(node->get_logger(), 
      "Could not get data port, using default: %i", serverDescription.dataPort);
  }
  
  if(!node->get_parameter(
    rosparam::keys::Version, 
    serverDescription.version)
  ) {
    RCLCPP_WARN(node->get_logger(), 
      "Could not get server version, using auto");
  }
  
  const std::vector<std::string> prefix{rosparam::keys::RigidBodies};
  const std::string name_with_dot = std::string(node->get_name()) + ".";
  const auto result = node->get_node_parameters_interface()->list_parameters(
    prefix, 0
  );

  for(const auto &prefix : result.prefixes)
  {
    PublisherConfiguration publisherConfig;

    publisherConfig.rigidBodyId = std::atoi(prefix.substr(rosparam::keys::RigidBodies.length() + 1, 1).c_str());

    const bool readPoseTopicName = node->get_parameter(
      prefix + "." + rosparam::keys::PoseTopicName, publisherConfig.poseTopicName);

    if (!readPoseTopicName)
    {
      RCLCPP_WARN(node->get_logger(), 
        "Failed to parse %s for body %d. Pose publishing disabled.", 
        rosparam::keys::PoseTopicName.c_str(), 
        publisherConfig.rigidBodyId);

      publisherConfig.publishPose = false;
    }
    else
    {
      publisherConfig.publishPose = true;
    }

    const bool readPose2dTopicName = node->get_parameter(
      prefix + "." + rosparam::keys::Pose2dTopicName, publisherConfig.pose2dTopicName);

    if (!readPose2dTopicName)
    {
      RCLCPP_WARN(node->get_logger(),
        "Failed to parse %s for body %d. Pose publishing disabled.",
          rosparam::keys::Pose2dTopicName.c_str(),
          publisherConfig.rigidBodyId);

      publisherConfig.publishPose2d = false;
    }
    else
    {
      publisherConfig.publishPose2d = true;
    }
  
    const bool readOdomTopicName =  node->get_parameter(prefix + "." + rosparam::keys::OdomTopicName,
                                                       publisherConfig.odomTopicName);

    if (!readOdomTopicName)
    {
      RCLCPP_WARN(node->get_logger(), "Failed to parse %s for body %d. Odom publishing disabled.",
                  rosparam::keys::OdomTopicName.c_str(), publisherConfig.rigidBodyId);
      publisherConfig.publishOdom = false;
    }
    else
    {
      publisherConfig.publishOdom = true;
    }

    const bool readEnableTfPublisher =  node->get_parameter(prefix + "." + rosparam::keys::EnableTfPublisher,
                                                           publisherConfig.enableTfPublisher);

    if (!readEnableTfPublisher)
    {
      RCLCPP_WARN(node->get_logger(), "Failed to parse %s for body %d. TF publishing disabled.",
                  rosparam::keys::EnableTfPublisher.c_str(), publisherConfig.rigidBodyId);
      publisherConfig.publishTf = false;
    }
    else
    {
      publisherConfig.publishTf = true;
    }

    const bool readChildFrameId = node->get_parameter(
      prefix + "." + rosparam::keys::ChildFrameId, publisherConfig.childFrameId);
      
    const bool readParentFrameId = node->get_parameter(
      prefix + "." + rosparam::keys::ParentFrameId, publisherConfig.parentFrameId);

    if (!readChildFrameId || !readParentFrameId)
    {
      if (!readChildFrameId)
        RCLCPP_WARN(node->get_logger(),
          "Failed to parse %s for body %d. TF publishing disabled.",
            rosparam::keys::ChildFrameId.c_str(),
            publisherConfig.rigidBodyId);

      if (!readParentFrameId)
        RCLCPP_WARN(node->get_logger(),
          "Failed to parse %s for body %d. TF publishing disabled.",
          rosparam::keys::ParentFrameId.c_str(),
          publisherConfig.rigidBodyId);

      publisherConfig.publishTf = false;
    }
    else
    {
      publisherConfig.publishTf = true;
    }

    pubConfigs.push_back(publisherConfig);
  }
}




}