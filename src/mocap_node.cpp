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

void processMocapData( const char** mocap_model )
{
  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );

  static tf::TransformBroadcaster br;
  static ros::NodeHandle n;
  static ros::Publisher pose_3d = n.advertise<geometry_msgs::Pose>("mocap_node/pose", 1000);
  static ros::Publisher pose_2d = n.advertise<geometry_msgs::Pose2D>("mocap_node/ground_pose", 1000);

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
        // Receive data form mocap device
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

            if( packetread && format.numModels > 0 )
            {
              ros::Time timestamp( ros::Time::now() );
                                
              for( int i = 0; i < format.model.numRigidBodies; i++ )
              {
                // publish 3D pose
                if(publish_pose)
                  pose_3d.publish(format.model.rigidBodies[i].pose);

                // publish 2D pose
                geometry_msgs::Pose2D pose;
                pose.x = format.model.rigidBodies[i].pose.position.x;
                pose.y = -format.model.rigidBodies[i].pose.position.z;

                tf::Quaternion q( format.model.rigidBodies[i].pose.orientation.x, format.model.rigidBodies[i].pose.orientation.z, format.model.rigidBodies[i].pose.orientation.y, format.model.rigidBodies[i].pose.orientation.w ) ;

                double roll, pitch, yaw;
                btMatrix3x3(q).getEulerYPR(yaw, pitch, roll);
                pose.theta = yaw;
                if(publish_ground_pose)
                  pose_2d.publish(pose);

                // publish transform
                tf::Transform transform;
                // Translate mocap data from mm --> m to be compatible with rviz
                transform.setOrigin( tf::Vector3(format.model.rigidBodies[i].pose.position.x / 1000.0f, format.model.rigidBodies[i].pose.position.y / 1000.0f, format.model.rigidBodies[i].pose.position.z / 1000.0f ) );

                transform.setRotation(q.inverse());             // Handle different coordinate systems (Arena vs. rviz)

                int rigid_body_id = abs(format.model.rigidBodies[i].ID);
                const char* rigid_body_name = mocap_model[rigid_body_id];
                if(publish_transform)
                  br.sendTransform(tf::StampedTransform(transform, timestamp, "base_link", std::string( rigid_body_name ) ));
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
  n.getParam(PUBLISH_TRANSFORM_KEY, publish_transform);
  n.getParam(PUBLISH_POSE_KEY, publish_pose);
  n.getParam(PUBLISH_GROUND_POSE_KEY, publish_ground_pose);

  // Process mocap data until SIGINT
  processMocapData( mocap_model );

  return 0;
}
