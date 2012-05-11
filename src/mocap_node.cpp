/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include "Socket.h"
#include "mocap_datapackets.h"
#include "mocap_config.h"
#include "skeletons.h"

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
const std::string MULTICAST_IP = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string PUBLISH_TRANSFORM_KEY = "publish_transform";
const std::string PUBLISH_POSE_KEY = "publish_pose";
const std::string PUBLISH_GROUND_POSE_KEY = "publish_ground_pose";
const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;
bool publish_transform = true;
bool publish_pose = true;
bool publish_ground_pose = true;

const int LOCAL_PORT = 1511;

////////////////////////////////////////////////////////////////////////

void processMocapData( const char** mocap_model, RigidBodyMap& published_rigid_bodies)
{
  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );

  ushort payload;
  while( true )
  {
    int numberOfPackets = 0;
    bool packetread = false;

    while( !packetread )
    {
      int numBytes = 0;

      do
      {
        // Receive data from mocap device
        numBytes = multicast_client_socket.recv();

        // Parse mocap data
        if( numBytes > 0 )
        {
          const char* buffer = multicast_client_socket.getBuffer();
          unsigned short header = *((unsigned short*)(&buffer[0]));

          // Look for the beginning of a NatNet package
          if (header == 7)
          {
            payload = *((ushort*) &buffer[2]);
            MoCapDataFormat format(buffer, payload);
            format.parse();
            packetread = true;
            numberOfPackets++;

            if( format.numModels > 0 )
            {
              for( int i = 0; i < format.model.numRigidBodies; i++ )
              {
                int ID = format.model.rigidBodies[i].ID;
                RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

                if (item != published_rigid_bodies.end())
                {
                    item->second.publish(format.model.rigidBodies[i]);
                }
              }
            }
          }
          // else skip packet
        }


      } while( numBytes > 0 );

      // Don't try again immediately
      if( !packetread )
      {
        usleep( 10 );
      }
    }
  }
}



////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{ 
  
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node", ros::init_options::NoSigintHandler );
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

  RigidBodyMap published_rigid_bodies;

  if (n.hasParam("rigid_bodies"))
  {
      XmlRpc::XmlRpcValue body_list;
      n.getParam("rigid_bodies", body_list);
      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedRigidBody body(i->second);
                  string id = (string&) (i->first);
                  RigidBodyItem item(atoi(id.c_str()), body);

                  std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
                  }
              }
          }
      }
  }

  n.getParam(PUBLISH_TRANSFORM_KEY, publish_transform);
  n.getParam(PUBLISH_POSE_KEY, publish_pose);
  n.getParam(PUBLISH_GROUND_POSE_KEY, publish_ground_pose);

  // Process mocap data until SIGINT
  processMocapData( mocap_model, published_rigid_bodies);

  return 0;
}
