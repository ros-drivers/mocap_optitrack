#ifndef __MOCAP_OPTITRACK_DATA_MODEL_H__
#define __MOCAP_OPTITRACK_DATA_MODEL_H__

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <mocap_optitrack/version.h>


namespace mocap_optitrack
{

/// \brief Data object holding the position of a single mocap marker in 3d space
struct Marker
{
  float positionX;
  float positionY;
  float positionZ;
};

struct __attribute__ ((__packed__)) Pose
{
  struct __attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
  } position;
  struct __attribute__ ((__packed__)) {
    float x;
    float y;
    float z;
    float w;
  } orientation;
};

/// \brief Data object holding information about a single rigid body within a mocap skeleton
struct RigidBody
{
    RigidBody();
    int bodyId;
    Pose pose;
    float meanMarkerError;
    bool isTrackingValid;

    const geometry_msgs::PoseStamped getRosPose(bool newCoordinates);
    bool hasData();
};

/// \brief Data object describing a single tracked model
struct ModelDescription
{
    ModelDescription();
    std::string name;
    int numMarkers;
    std::vector<std::string> markerNames;
};

struct MarkerSet
{
    MarkerSet();
    char name[256];
    int numMarkers;
    std::vector<Marker> markers;
};

/// \brief Data object holding poses of a tracked model's components
struct ModelFrame
{
    ModelFrame();
    std::vector<MarkerSet> markerSets;
    std::vector<Marker> otherMarkers;
    std::vector<RigidBody> rigidBodies;

    float latency;
};

/// \brief Data object holding server info
struct ServerInfo
{
    ServerInfo();
    Version natNetVersion;
    Version serverVersion;
};

}

#endif