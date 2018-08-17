#include "natnet_messages.h"

#include "natnet_packet_definition.h"

namespace natnet
{

namespace utilities
{
    template<typename T>
    struct TypeParseTraits {};

    #define REGISTER_PARSE_TYPE(X)                    \
    template <>                                       \
    struct TypeParseTraits<X>                         \
    {                                                 \
      static const char* name;                        \
    };                                                \
    const char* TypeParseTraits<X>::name = #X;

    REGISTER_PARSE_TYPE(int);
    REGISTER_PARSE_TYPE(double);
    REGISTER_PARSE_TYPE(float);
    REGISTER_PARSE_TYPE(short);
    // REGISTER_PARSE_TYPE(Pose);

    void seek(char* packet, size_t offset)
    {
      packet += offset;
    }

    template <typename T> 
    void read_and_seek(char* packet, T& target)
    {
      target = *((T*) packet);
      ROS_DEBUG("\t sizeof(%s) = %d", TypeParseTraits<T>::name, sizeof(T));
      seek(packet, sizeof(T));
    }
}

size_t ConnectionRequestMessage::length()
{
    // 2 bytes for messageId and 2 for numDataBtyes
    return 4;
}

void ConnectionRequestMessage::serialize(char* msgBuffer)
{
    natnet::Packet pkt;
    pkt.messageId = natnet::MessageType::Connect;
    pkt.numDataBytes = 0;
    memcpy(msgBuffer, &pkt, length());
}

void ServerInfoMessage::unserialize(const char* msgBuffer, 
    mocap_optitrack::ServerInfo &serverInfo)
{
    const natnet::Packet* packet = (natnet::Packet const*)msgBuffer;

    int nver[4] = {0, 0, 0, 0};
    int sver[4] = {0, 0, 0, 0};
    for(int i=0;i<4;++i) {
        nver[i] = (int)(packet->data.sender.natNetVersion[i]);
        sver[i] = (int)(packet->data.sender.version[i]);
    }

    serverInfo.natNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
    serverInfo.serverVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
}

// void DataFrameMessage::unserialize(const char* msgBuffer, mocap_optitrack::ModelFrame& modelFrame)
// {
//   // Next 4 Bytes is the frame numberc
//   read_and_seek(packet, frameNumber);
//   ROS_DEBUG("Frame number: %d", frameNumber);

//   // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
//   read_and_seek(packet, model.numMarkerSets);
//   model.markerSets = new MarkerSet[model.numMarkerSets];
//   ROS_DEBUG("Number of marker sets: %d", model.numMarkerSets);

  // Loop through number of marker sets and get name and data
  // for (int i = 0; i < model.numMarkerSets; i++)
  // {
  //   // Markerset name
  //   strcpy(model.markerSets[i].name, packet);
  //   seek(packet, strlen(model.markerSets[i].name) + 1);
  //   ROS_DEBUG("Parsing marker set named: %s", model.markerSets[i].name);

  //   // read number of markers that belong to the model
  //   read_and_seek(packet, model.markerSets[i].numMarkers);
  //   ROS_DEBUG("Number of markers in set: %d", model.markerSets[i].numMarkers);
  //   model.markerSets[i].markers = new Marker[model.markerSets[i].numMarkers];

  //   for (int k = 0; k < model.markerSets[i].numMarkers; k++)
  //   {
  //     // read marker positions
  //     read_and_seek(packet, model.markerSets[i].markers[k]);
  //     float x = model.markerSets[i].markers[k].positionX;
  //     float y = model.markerSets[i].markers[k].positionY;
  //     float z = model.markerSets[i].markers[k].positionZ;
  //     ROS_DEBUG("\t marker %d: [x=%3.2f,y=%3.2f,z=%3.2f]", k, x, y, z);
  //   }
  // }

  // // Loop through unlabeled markers
  // read_and_seek(packet, model.numOtherMarkers);
  // model.otherMarkers = new Marker[model.numOtherMarkers];
  // ROS_DEBUG("Number of markers not in sets: %d", model.numOtherMarkers);

  // for (int l = 0; l < model.numOtherMarkers; l++)
  // {
  //   // read positions of 'other' markers
  //   read_and_seek(packet, model.otherMarkers[l]);
  //   // Deprecated
  // }

  // // Loop through rigidbodies
  // read_and_seek(packet, model.numRigidBodies);
  // ROS_DEBUG("Number of rigid bodies: %d", model.numRigidBodies);

  // model.rigidBodies = new RigidBody[model.numRigidBodies];
  // for (int m = 0; m < model.numRigidBodies; m++)
  // {
  //   parseRigidBody(model.rigidBodies[m]);
  // } // Go to next rigid body

  // // Skeletons (NatNet version 2.1 and later)
  // if (NatNetVersion >= Version("2.1"))
  // {
  //   int nSkeletons = 0;
  //   read_and_seek(packet, nSkeletons);
  //   ROS_DEBUG("Skeleton count: %d", nSkeletons);

  //   // Loop through skeletons
  //   for (int j=0; j < nSkeletons; j++)
  //   {
  //     // skeleton id
  //     int skeletonID = 0;
  //     read_and_seek(packet, skeletonID);
  //     ROS_DEBUG("Skeleton ID: %d", skeletonID);

  //     // Number of rigid bodies (bones) in skeleton
  //     int nRigidBodies = 0;
  //     read_and_seek(packet, nRigidBodies);
  //     ROS_DEBUG("Rigid body count: %d", nRigidBodies);

  //     // Loop through rigid bodies (bones) in skeleton
  //     for (int j=0; j < nRigidBodies; j++)
  //     {
  //       RigidBody rigidBody;
  //       parseRigidBody(rigidBody);
  //     } // next rigid body
  //   } // next skeleton
  // }

  // // labeled markers (NatNet version 2.3 and later)
  // if (NatNetVersion >= Version("2.3"))
  // {
  //   int nLabeledMarkers = 0;
  //   read_and_seek(packet, nLabeledMarkers);
  //   ROS_DEBUG("Labeled marker count : %d", nLabeledMarkers);

  //   // Loop through labeled markers
  //   for (int j=0; j < nLabeledMarkers; j++)
  //   {
  //     // id
  //     // Marker ID Scheme:
  //     // Active Markers:
  //     //   ID = ActiveID, correlates to RB ActiveLabels list
  //     // Passive Markers: 
  //     //   If Asset with Legacy Labels
  //     //      AssetID   (Hi Word)
  //     //      MemberID  (Lo Word)
  //     //   Else
  //       //      PointCloud ID
  //     int ID = 0; 
  //     read_and_seek(packet, ID);
  //     int modelID, markerID;
  //     decodeMarkerID(ID, &modelID, &markerID);

  //     Marker marker;
  //     read_and_seek(packet, marker);
      
  //     float size;
  //     read_and_seek(packet, size);

  //     if (NatNetVersion >= Version("2.6"))
  //     {
  //       // marker params
  //       short params = 0;
  //       read_and_seek(packet, params);
  //       // marker was not visible (occluded) in this frame
  //       bool bOccluded = (params & 0x01) != 0;
  //       // position provided by point cloud solve     
  //       bool bPCSolved = (params & 0x02) != 0;
  //       // position provided by model solve
  //       bool bModelSolved = (params & 0x04) != 0;  
  //       if (NatNetVersion >= Version("3.0"))
  //       {
  //         // marker has an associated model
  //         bool bHasModel = (params & 0x08) != 0;
  //         // marker is an unlabeled marker
  //         bool bUnlabeled = (params & 0x10) != 0;   
  //         // marker is an active marker 
  //         bool bActiveMarker = (params & 0x20) != 0;
  //       }
  //     }

  //     ROS_DEBUG("MarkerID: %d, ModelID: %d", markerID, modelID);
  //     ROS_DEBUG("\tpos: [%3.2f,%3.2f,%3.2f]", marker.positionX, 
  //       marker.positionY, marker.positionZ);
  //     ROS_DEBUG("\tsize: %3.2f", size);

  //     // NatNet version 3.0 and later
  //     if (NatNetVersion >= Version("3.0"))
  //     {
  //       // Marker residual
  //       float residual = 0.0f;
  //       read_and_seek(packet, residual);
  //       ROS_DEBUG("\terr:  %3.2f", residual);
  //     }
  //   }
  // }

  // // Force Plate data (NatNet version 2.9 and later)
  // if (NatNetVersion >= Version("2.9"))
  // {
  //     int nForcePlates;
  //     read_and_seek(packet, nForcePlates);
  //     for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
  //     {
  //         // ID
  //         int forcePlateID = 0;
  //         read_and_seek(packet, forcePlateID);
  //         ROS_DEBUG("Force plate ID: %d", forcePlateID);

  //         // Channel Count
  //         int nChannels = 0; 
  //         read_and_seek(packet, nChannels);
  //         ROS_DEBUG("Number of channels: %d", nChannels);

  //         // Channel Data
  //         for (int i = 0; i < nChannels; i++)
  //         {
  //             ROS_DEBUG("\tchannel %d: ", i);
  //             int nFrames = 0;
  //             read_and_seek(packet, nFrames);
  //             for (int j = 0; j < nFrames; j++)
  //             {
  //                 float val = 0.0f;  
  //                 read_and_seek(packet, val);
  //                 ROS_DEBUG("\t\tframe %d: %3.2f", j, val);
  //             }
  //         }
  //     }
  // }

  // // Device data (NatNet version 3.0 and later)
  // if (NatNetVersion >= Version("3.0"))
  // {
  //   int nDevices;
  //   read_and_seek(packet, nDevices);
  //   for (int iDevice = 0; iDevice < nDevices; iDevice++)
  //   {
  //     // ID
  //     int deviceID = 0;
  //     read_and_seek(packet, deviceID);
  //     ROS_DEBUG("Device ID: %d", deviceID);

  //     // Channel Count
  //     int nChannels = 0;
  //     read_and_seek(packet, nChannels);

  //     // Channel Data
  //     for (int i = 0; i < nChannels; i++)
  //     {
  //       ROS_DEBUG("\tchannel %d : ", i);
  //       int nFrames = 0; 
  //       read_and_seek(packet, nFrames);
  //       for (int j = 0; j < nFrames; j++)
  //       {
  //           float val = 0.0f;
  //           read_and_seek(packet, val);
  //           ROS_DEBUG("\t\tframe %d: %3.2f", j, val);
  //       }
  //     }
  //   }
  // }

  // // software latency (removed in version 3.0)
  // if (NatNetVersion < Version("3.0"))
  // {
  //   float softwareLatency = 0.0f; 
  //   read_and_seek(packet, softwareLatency);
  //   ROS_DEBUG("Software latency : %3.3f", softwareLatency);
  // }

  // // timecode
  // unsigned int timecode = 0;
  // read_and_seek(packet, timecode);
  // unsigned int timecodeSub = 0;
  // read_and_seek(packet, timecodeSub);
  // char szTimecode[128] = "";
  // timecodeStringify(timecode, timecodeSub, szTimecode, 128);

  // // timestamp
  // double timestamp = 0.0f;

  // // NatNet version 2.7 and later - increased from single to double precision
  // if (NatNetVersion >= Version("2.7"))
  // {
  //   read_and_seek(packet, timestamp);
  // }
  // else
  // {
  //   float fTimestamp = 0.0f;
  //   read_and_seek(packet, fTimestamp);
  //   timestamp = (double)fTimestamp;
  // }
  // ROS_DEBUG("Timestamp: %3.3f", timestamp);

  // // high res timestamps (version 3.0 and later)
  // if (NatNetVersion >= Version("3.0"))
  // {
  //   uint64_t cameraMidExposureTimestamp = 0;
  //   read_and_seek(packet, cameraMidExposureTimestamp);
  //   ROS_DEBUG("Mid-exposure timestamp : %" PRIu64 "", cameraMidExposureTimestamp);

  //   uint64_t cameraDataReceivedTimestamp = 0;
  //   read_and_seek(packet, cameraDataReceivedTimestamp);
  //   ROS_DEBUG("Camera data received timestamp: %" PRIu64 "", cameraDataReceivedTimestamp);

  //   uint64_t transmitTimestamp = 0;
  //   read_and_seek(packet, transmitTimestamp);
  //   ROS_DEBUG("Transmit timestamp: %" PRIu64 "", transmitTimestamp);
  // }

  // // frame params
  // short params = 0;  
  // read_and_seek(packet, params);
  // // 0x01 Motive is recording
  // bool bIsRecording = (params & 0x01) != 0;
  // // 0x02 Actively tracked model list has changed
  // bool bTrackedModelsChanged = (params & 0x02) != 0;

  // // end of data tag
  // int eod = 0; 
  // read_and_seek(packet, eod);
  // ROS_DEBUG("------------- End Packet -------------");
// }

}