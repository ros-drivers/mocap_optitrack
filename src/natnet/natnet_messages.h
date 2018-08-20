#ifndef __MOCAP_OPTITRACK_NATNET_MESSAGES_H__
#define __MOCAP_OPTITRACK_NATNET_MESSAGES_H__

#include <vector>
#include <mocap_optitrack/data_model.h>

namespace natnet
{
    typedef std::vector<char> MessageBuffer;

    struct MessageInterface
    {
        virtual void serialize(MessageBuffer&, mocap_optitrack::DataModel const*) {};
        virtual void deserialize(MessageBuffer const&, mocap_optitrack::DataModel*) {};
    };

    struct ConnectionRequestMessage : public MessageInterface
    {
        virtual void serialize(MessageBuffer& msgBuffer, mocap_optitrack::DataModel const*);
    };

    struct ServerInfoMessage : public MessageInterface
    {
        virtual void deserialize(MessageBuffer const&, mocap_optitrack::DataModel*);
    };

    class DataFrameMessage : public MessageInterface
    {
        struct RigidBodyMessagePart
        {
            void deserialize(MessageBuffer::const_iterator&, 
                mocap_optitrack::RigidBody&,
                mocap_optitrack::Version const&);
        };

    public:
        virtual void deserialize(MessageBuffer const&, mocap_optitrack::DataModel*);
    };

    struct MessageDispatcher
    {
        static void dispatch(MessageBuffer const&, mocap_optitrack::DataModel*);
    };
}

#endif