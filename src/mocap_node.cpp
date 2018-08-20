/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include <mocap_optitrack/socket.h>
#include <mocap_optitrack/data_model.h>
#include <mocap_optitrack/mocap_config.h>
#include <mocap_optitrack/rigid_body_publisher.h>
#include "natnet/natnet_messages.h"

// ROS includes
#include <ros/ros.h>


namespace mocap_optitrack
{

  class OptiTrackRosBridge
  {
  public:
    OptiTrackRosBridge(ros::NodeHandle& nh,
      ServerDescription const& serverDescr, 
      PublisherConfigurations const& pubConfigs) :
        nh(nh),
        serverDescription(serverDescr),
        publisherConfigurations(pubConfigs)
    {

    }

    void initialize()
    {
      // Create socket
      multicastClientSocketPtr.reset(
        new UdpMulticastSocket(serverDescription.dataPort, 
          serverDescription.multicastIpAddress)); 

      // Need verion information from the server to properly decode any of their packets.
      // If we have not recieved that yet, send another request.  
      while(ros::ok() && !dataModel.hasServerInfo())
      {
        natnet::ConnectionRequestMessage connectionRequestMsg;
        natnet::MessageBuffer connectionRequestMsgBuffer;
        connectionRequestMsg.serialize(connectionRequestMsgBuffer, NULL);
        int ret = multicastClientSocketPtr->send(
          &connectionRequestMsgBuffer[0], 
          connectionRequestMsgBuffer.size(), 
          serverDescription.commandPort);

        if (updateDataModelFromServer()) usleep(10);
      }

      // Once we have the server info, create publishers
      publishDispatcherPtr.reset(
        new RigidBodyPublishDispatcher(nh, 
          dataModel.getNatNetVersion(), 
          publisherConfigurations));

      ROS_INFO("Initialization complete");
    };

    void run()
    {
      while (ros::ok())
      {
        if (updateDataModelFromServer())
        {
          // Maybe we got some data? If we did it would be in the form of one or more
          // rigid bodies in the data model
          ros::Time time = ros::Time::now();
          publishDispatcherPtr->publish(time, dataModel.dataFrame.rigidBodies);

          // Clear out the model to prepare for the next frame of data
          dataModel.clear();

          // If we processed some data, take a short break
          usleep( 10 );
        }
      }
    }

  private:
    bool updateDataModelFromServer()
    {
      // Get data from mocap server
      int numBytesReceived = multicastClientSocketPtr->recv();
      if( numBytesReceived > 0 )
      {
        // Grab latest message buffer
        const char* pMsgBuffer = multicastClientSocketPtr->getBuffer();

        // Copy char* buffer into MessageBuffer and dispatch to be deserialized
        natnet::MessageBuffer msgBuffer(pMsgBuffer, pMsgBuffer + numBytesReceived);
        natnet::MessageDispatcher::dispatch(msgBuffer, &dataModel);

        return true;
      }

      return false;
    };

    ros::NodeHandle& nh;
    ServerDescription serverDescription;
    PublisherConfigurations publisherConfigurations;
    DataModel dataModel;
    std::unique_ptr<UdpMulticastSocket> multicastClientSocketPtr;
    std::unique_ptr<RigidBodyPublishDispatcher> publishDispatcherPtr;
  };

} // namespace


////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle nh("~");

  // Grab node configuration from rosparam
  mocap_optitrack::ServerDescription serverDescription;
  mocap_optitrack::PublisherConfigurations publisherConfigurations;
  mocap_optitrack::NodeConfiguration::fromRosParam(nh, serverDescription, publisherConfigurations);

  // Create node object, initialize and run
  mocap_optitrack::OptiTrackRosBridge node(nh, serverDescription, publisherConfigurations);
  node.initialize();
  node.run();

  return 0;
}
