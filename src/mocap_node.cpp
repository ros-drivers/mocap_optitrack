/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include "mocap_optitrack/socket.h"
// #include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/data_model.h"
#include "natnet_messages.h"
#include "natnet_packet_definition.h"
#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/skeletons.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

// System includes
#include <string>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP_KEY = "optitrack_config/multicast_address";
const std::string MULTICAST_IP_DEFAULT = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char ** DEFAULT_MOCAP_MODEL = OBJECT;
//const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int COMMAND_PORT = 1510;
const int LOCAL_PORT = 9000;


namespace mocap_optitrack
{
  ////////////////////////////////////////////////////////////////////////

  void processMocapData( const char** mocap_model,
                         RigidBodyMap& published_rigid_bodies,
                         const std::string& multicast_ip)
  {
    UdpMulticastSocket multicast_client_socket( LOCAL_PORT, multicast_ip );

    ROS_INFO("Start processMocapData");
    mocap_optitrack::DataModel dataModel;
    natnet::MessageDispatcher msgDispatcher;
    while(ros::ok())
    {
      // Need verion information from the server to properly decode any of their packets.
      // If we have not recieved that yet, send another request.
      if (!dataModel.hasServerInfo())
      {
        natnet::ConnectionRequestMessage connectionRequestMsg;
        natnet::MessageBuffer connectionRequestMsgBuffer;
        connectionRequestMsg.serialize(connectionRequestMsgBuffer, NULL);
        int ret = multicast_client_socket.send(&connectionRequestMsgBuffer[0], connectionRequestMsgBuffer.size(), COMMAND_PORT);
      }

      // Receive data from mocap device
      int numBytesReceived = multicast_client_socket.recv();

      // Parse mocap data
      if( numBytesReceived > 0 )
      {
        // Grab latest message buffer
        const char* pMsgBuffer = multicast_client_socket.getBuffer();

        // Copy char* buffer into MessageBuffer and dispatch to be deserialized
        natnet::MessageBuffer msgBuffer(pMsgBuffer, pMsgBuffer + numBytesReceived);
        msgDispatcher.dispatch(msgBuffer, &dataModel);

        // Maybe we got some data? Publish what we can to ROS
        for (auto const& rigidBody : dataModel.dataFrame.rigidBodies)
        {
          RigidBodyMap::iterator item = published_rigid_bodies.find(rigidBody.bodyId);

          if (item != published_rigid_bodies.end())
          {
            item->second.publish(rigidBody);
          }
        }

        // Clear out the model to prepare for the next frame of data
        dataModel.clear();
        
        // If we processed some data, take a short break
        usleep( 10 );
      }
    } // while ros::ok
  }
}


////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n("~");

  // Get configuration from ROS parameter server
  const char** mocap_model( DEFAULT_MOCAP_MODEL );
  if( n.hasParam( MOCAP_MODEL_KEY ) )
  {    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )
    {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
    }
  }

  // Get configuration from ROS parameter server
  std::string multicast_ip( MULTICAST_IP_DEFAULT );
  if( n.hasParam( MULTICAST_IP_KEY ) )
  {
    n.getParam( MULTICAST_IP_KEY, multicast_ip );
  }
  else {
    ROS_WARN_STREAM("Could not get multicast address, using default: " << multicast_ip);
  }

  mocap_optitrack::RigidBodyMap published_rigid_bodies;

  if (n.hasParam(RIGID_BODIES_KEY))
  {
      XmlRpc::XmlRpcValue body_list;
      n.getParam("rigid_bodies", body_list);
      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  mocap_optitrack::PublishedRigidBody body(i->second);
                  std::string id = (std::string&) (i->first);
                  mocap_optitrack::RigidBodyItem item(atoi(id.c_str()), body);

                  std::pair<mocap_optitrack::RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
                  }
              }
          }
      }
  }

  // Process mocap data until SIGINT
  mocap_optitrack::processMocapData(mocap_model, published_rigid_bodies, multicast_ip);

  return 0;
}
