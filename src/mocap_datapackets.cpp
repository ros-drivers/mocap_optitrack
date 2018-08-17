#include "mocap_optitrack/mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdint>
#include <cinttypes>
#include <ros/console.h>

#include "natnet_packet_definition.h"

using namespace std;


RigidBody::RigidBody()
{
}

RigidBody::~RigidBody()
{
}

const geometry_msgs::PoseStamped RigidBody::get_ros_pose(bool newCoordinates)
{
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.header.stamp = ros::Time::now();
  if (newCoordinates)
  {
    // Motive 1.7+ coordinate system
    ros_pose.pose.position.x = -pose.position.x;
    ros_pose.pose.position.y = pose.position.z;
    ros_pose.pose.position.z = pose.position.y;

    ros_pose.pose.orientation.x = -pose.orientation.x;
    ros_pose.pose.orientation.y = pose.orientation.z;
    ros_pose.pose.orientation.z = pose.orientation.y;
    ros_pose.pose.orientation.w = pose.orientation.w;
  }
  else
  {
    // y & z axes are swapped in the Optitrack coordinate system
    ros_pose.pose.position.x = pose.position.x;
    ros_pose.pose.position.y = -pose.position.z;
    ros_pose.pose.position.z = pose.position.y;

    ros_pose.pose.orientation.x = pose.orientation.x;
    ros_pose.pose.orientation.y = -pose.orientation.z;
    ros_pose.pose.orientation.z = pose.orientation.y;
    ros_pose.pose.orientation.w = pose.orientation.w;
  }
  return ros_pose;
}

bool RigidBody::has_data()
{
    static const char zero[sizeof(pose)] = { 0 };
    return memcmp(zero, (char*) &pose, sizeof(pose));
}

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markerSets(0), otherMarkers(0), rigidBodies(0),
    numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0),
    latency(0.0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markerSets;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

NatNetParser::NatNetParser()
  : haveVersion(false)
{
}

NatNetParser::~NatNetParser()
{
}

void NatNetParser::parse(const char* msgBuffer, int numBytes)
{
  // Make a local copy of incoming message
  natnet::Packet packet;
  memcpy((char*)&packet, msgBuffer, numBytes);
  ROS_DEBUG("Message ID: %d", packet.messageId);
  ROS_DEBUG("Byte count : %d", numBytes);

  // Handle the message type
  switch (packet.messageId)
  {
    case natnet::MessageType::ModelDef:
    case natnet::MessageType::FrameOfData:
      if (haveVersion)
      {
        parseMocapDataFrame(packet);
      }
      else
      {
        ROS_WARN("Packet version not received from server. Parsing data message aborted.");
      }
      break;
    case natnet::MessageType::ServerInfo:
      int nver[4] = {0, 0, 0, 0};
      int sver[4] = {0, 0, 0, 0};
      for(int i=0;i<4;++i) {
        nver[i] = (int)PacketIn.Data.Sender.NatNetVersion[i];
        sver[i] = (int)PacketIn.Data.Sender.Version[i];
      }
      setVersion(nver, sver);

      ROS_INFO_ONCE("NATNet Version : %d.%d.%d.%d", nver[0], nver[1], nver[2], nver[3]);
      ROS_INFO_ONCE("Server Version : %d.%d.%d.%d", sver[0], sver[1], sver[2], sver[3]);
      haveVersion = true;
      break;
    case natnet::MessageType::UnrecognizedRequest:
    case natnet::MessageType::Unknown:
      ROS_WARN("Received unrecognized request");
      break;
  }

  // // // First 2 Bytes is message ID
  // short MessageID = 0;
  // read_and_seek(packet, MessageID);
  // ROS_DEBUG("Message ID: %d", MessageID);

  // // Second 2 Bytes is the size of the packet
  // short nBytes = 0;
  // read_and_seek(packet, nBytes);
  // ROS_DEBUG("Byte count : %d", nBytes);
  // seek(packet, 4);

  // if (MessageID == 7)
  // {
    // parseMocapDataFrame();
  // }
  // TODO: implement parsing of other message IDs
}

void NatNetParser::decodeMarkerID(int sourceID, int* pOutEntityID, int* pOutMemberID)
{
    if (pOutEntityID)
        *pOutEntityID = sourceID >> 16;

    if (pOutMemberID)
        *pOutMemberID = sourceID & 0x0000ffff;
}

// Funtion that assigns a time code values to 5 variables passed as arguments
// Requires an integer from the packet as the timecode and timecodeSubframe
bool NatNetParser::decodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe)
{
  bool bValid = true;

  *hour = (inTimecode>>24)&255;
  *minute = (inTimecode>>16)&255;
  *second = (inTimecode>>8)&255;
  *frame = inTimecode&255;
  *subframe = inTimecodeSubframe;

  return bValid;
}

// Takes timecode and assigns it to a string
bool NatNetParser::timecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize)
{
  bool bValid;
  int hour, minute, second, frame, subframe;
  bValid = decodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

  snprintf(Buffer,BufferSize,"%2d:%2d:%2d:%2d.%d",hour, minute, second, frame, subframe);
  for(unsigned int i=0; i < strlen(Buffer); i++)
  {
    if(Buffer[i]==' ')
    {
      Buffer[i]='0';
    }
  }

  return bValid;
}

void NatNetParser::parseRigidBody(RigidBody &rigidBody)
{
  // Read id, position and orientation of each rigid body
  read_and_seek(packet, rigidBody.ID);
  read_and_seek(packet, rigidBody.pose);

  ROS_DEBUG("Rigid body ID: %d", rigidBody.ID);
  ROS_DEBUG("pos: [%3.2f,%3.2f,%3.2f], ori: [%3.2f,%3.2f,%3.2f,%3.2f]",
           rigidBody.pose.position.x,
           rigidBody.pose.position.y,
           rigidBody.pose.position.z,
           rigidBody.pose.orientation.x,
           rigidBody.pose.orientation.y,
           rigidBody.pose.orientation.z,
           rigidBody.pose.orientation.w);

  // NatNet version 2.0 and later
  if (NatNetVersion >= Version("2.0"))
  {
    // Mean marker error
    read_and_seek(packet, rigidBody.meanMarkerError);
    ROS_DEBUG("Mean marker error: %3.2f\n", rigidBody.meanMarkerError);
  }

  // NatNet version 2.6 and later
  if (NatNetVersion >= Version("2.6"))
  {
    // params
    short params = 0; 
    read_and_seek(packet, params);
    rigidBody.isTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
    ROS_DEBUG("Rigid body successfully tracked in this frame: %s", 
      (rigidBody.isTrackingValid ? "YES" : "NO"));
  }
}

