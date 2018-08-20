#include "natnet_packet_definition.h"

namespace natnet
{
  const int MessageType::Connect             = 0;
  const int MessageType::ServerInfo          = 1;
  const int MessageType::Request             = 2;
  const int MessageType::Response            = 3;
  const int MessageType::RequestModelDef     = 4;
  const int MessageType::ModelDef            = 5;
  const int MessageType::RequestFrameOfData  = 6;
  const int MessageType::FrameOfData         = 7;
  const int MessageType::MessageString       = 8;
  const int MessageType::UnrecognizedRequest = 100;
  const int MessageType::Undefined           = 999999;
}