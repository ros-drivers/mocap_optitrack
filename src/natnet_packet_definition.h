#ifndef __NAT_SDK_PACKET_DEFINITION_H__
#define __NAT_SDK_PACKET_DEFINITION_H__

#define MAX_NAMELENGTH              256
#define MAX_PACKETSIZE              100000  // max size of packet (actual packet size is dynamic)

namespace natnet
{
  struct MessageType
  {
    static const int Connect;
    static const int ServerInfo;
    static const int Request;
    static const int Response;
    static const int RequestModelDef;
    static const int ModelDef;
    static const int RequestFrameOfData;
    static const int FrameOfData;
    static const int MessageString;
    static const int UnrecognizedRequest;
    static const int Undefined;
  };

  // Sender definition
  typedef struct 
  {
    char name[MAX_NAMELENGTH];            // sending app's name
    unsigned char version[4];             // sending app's version [major.minor.build.revision]
    unsigned char natNetVersion[4];       // sending app's NatNet version [major.minor.build.revision]
  } __attribute__ ((__packed__)) Sender;
  
  // Packet definition
  typedef struct
  {
    unsigned short messageId;               // message ID (e.g. NAT_FRAMEOFDATA)
    unsigned short numDataBytes;            // Num bytes in payload
    union
    {
      unsigned char  cData[MAX_PACKETSIZE];
      char           szData[MAX_PACKETSIZE];
      unsigned long  lData[MAX_PACKETSIZE/4];
      float          fData[MAX_PACKETSIZE/4];
      Sender         sender;
    } data;                                 // Payload incoming from NatNet Server
  } __attribute__ ((__packed__)) Packet;
}

#endif