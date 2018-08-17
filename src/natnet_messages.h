#ifndef __MOCAP_OPTITRACK_NATNET_MESSAGES_H__
#define __MOCAP_OPTITRACK_NATNET_MESSAGES_H__

#include <mocap_optitrack/data_model.h>

namespace natnet
{
    struct ConnectionRequestMessage
    {
        size_t length();
        void serialize(char* msgBuffer);
    };

    struct ServerInfoMessage
    {
        void unserialize(const char* msgBuffer, 
            mocap_optitrack::ServerInfo &serverInfo);
    };

    // struct DataFrameMessage
    // {
    //     void unserialize(const char* msgBuffer, mocap_optitrack::ModelFrame& modelFrame);
    // };
}

#endif