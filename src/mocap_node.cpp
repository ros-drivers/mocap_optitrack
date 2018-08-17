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

// NATNET message ids
#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999
#define MAX_PACKETSIZE              100000  // max size of packet (actual packet size is dynamic)
#define MAX_NAMELENGTH              256

namespace mocap_optitrack
{
  ////////////////////////////////////////////////////////////////////////

  void processMocapData( const char** mocap_model,
                         RigidBodyMap& published_rigid_bodies,
                         const std::string& multicast_ip)
  {
    UdpMulticastSocket multicast_client_socket( LOCAL_PORT, multicast_ip );

    unsigned short payload_len;
    int numberOfPackets = 0;

    natnet::ConnectionRequestMessage connectionRequestMsg;
    char connectionRequestMsgBuffer[MAX_PACKETSIZE];
    connectionRequestMsg.serialize(connectionRequestMsgBuffer);

    ROS_INFO("Start processMocapData");
    mocap_optitrack::ServerInfo serverInfo;
    bool haveServerInfo = false;

    while(ros::ok())
    {
      int numBytes;

      if (!haveServerInfo)
      {
        int ret = multicast_client_socket.send(connectionRequestMsgBuffer, connectionRequestMsg.length(), COMMAND_PORT);
      }

      do
      {
        // Receive data from mocap device
        numBytes = multicast_client_socket.recv();

        // Parse mocap data
        if( numBytes > 0 )
        {
          const char* msgBuffer = multicast_client_socket.getBuffer();
          const natnet::Packet* packet = (natnet::Packet const*)(msgBuffer);
          // ROS_DEBUG("Message ID: %d", packet->messageId);
          // ROS_DEBUG("Byte count : %d", numBytes);

          if (packet->messageId == natnet::MessageType::ModelDef ||
              packet->messageId == natnet::MessageType::FrameOfData)
          {
            // if (haveVersion)
            // {
            //   parseMocapDataFrame(packet);
            // }
            // else
            // {
            //   ROS_WARN("Packet version not received from server. Parsing data message aborted.");
            // }
            continue;
          }

          if (packet->messageId == natnet::MessageType::ServerInfo)
          {
            natnet::ServerInfoMessage msg;
            msg.unserialize(msgBuffer, serverInfo);
            ROS_INFO_ONCE("NATNet Version : %s", 
              serverInfo.natNetVersion.getVersionString().c_str());
            ROS_INFO_ONCE("Server Version : %s", 
              serverInfo.serverVersion.getVersionString().c_str());
            haveServerInfo = true;
            continue;
          }

          if (packet->messageId == natnet::MessageType::UnrecognizedRequest)
          {
            ROS_WARN("Received unrecognized request");
          }
        }
      } while (numBytes > 0);

      // NatNetParser natNetParser;
      // bool packetread = false;

      // if(!version) {
      //   int iRet = multicast_client_socket.send((char*)&PacketOut, 4 + PacketOut.nDataBytes, COMMAND_PORT);
      // }

      // do
      // {
      //   // Receive data from mocap device
      //   numBytes = multicast_client_socket.recv();

      //   // Parse mocap data
      //   if( numBytes > 0 )
      //   {
      //     const char* buffer = multicast_client_socket.getBuffer();
          
      //   }
      // } while( numBytes > 0 );

      // Don't try again immediately
      // if( !packetread )
      // {
        usleep( 10 );
      // }
    }
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
