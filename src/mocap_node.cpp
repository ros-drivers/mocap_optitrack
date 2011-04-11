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
#include <tf/transform_broadcaster.h>

// System includes
#include <string>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int LOCAL_PORT = 1001;

////////////////////////////////////////////////////////////////////////

void processMocapData( const char** mocap_model )
{
  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );
		
  static tf::TransformBroadcaster br;

  ushort payload;
  MoCapDataFormat format;
  while( true )
  {	
    int numberOfPackets = 0;
    bool packetread = false;

    int trials = 0;

    while( !packetread && trials < 100 )
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
	    format.parse(buffer,payload);
	    packetread = true;
	    numberOfPackets++;
	  }
	  // else skip packet
	}


      } while( numBytes > 0 );

      // Don't try again immediately
      if( !packetread )
      {
	usleep( 10 );
	trials++;
      }
    }
		
    // Once a mocap package has been received and parsed, publish the data using tf
    if( packetread )
    {
      ros::Time timestamp( ros::Time::now() );
			
      for( int i = 0; i < format.model[0].numRigidBodies; i++ )
      {
	tf::Transform transform;
	// Translate mocap data from mm --> m to be compatible with rviz
	transform.setOrigin( tf::Vector3(format.model[0].rigidBodies[i].positionX / 1000.0f,format.model[0].rigidBodies[i].positionY / 1000.0f,format.model[0].rigidBodies[i].positionZ / 1000.0f ) );
			    
	tf::Quaternion q( format.model[0].rigidBodies[i].quaternionX, format.model[0].rigidBodies[i].quaternionY, format.model[0].rigidBodies[i].quaternionZ,format.model[0].rigidBodies[i].quaternionW ) ;
			    
	transform.setRotation(q.inverse());		// Handle different coordinate systems (Arena vs. rviz)
			    
	int rigid_body_id = abs(format.model[0].rigidBodies[i].ID);
	const char* rigid_body_name = mocap_model[rigid_body_id];
	br.sendTransform(tf::StampedTransform(transform, timestamp, "base_link", std::string( rigid_body_name ) ));
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
  
  // Process mocap data until SIGINT
  processMocapData( mocap_model );

  return 0;
}
