#include "mocap_optitrack/data_model.h"

namespace mocap_optitrack
{

RigidBody::RigidBody() : 
  isTrackingValid(false)
{
}

bool RigidBody::hasValidData() const
{
    return isTrackingValid;
}


void ModelDescription::clear()
{
  markerNames.clear();
}

void MarkerSet::clear()
{
  markers.clear();
}


ModelFrame::ModelFrame() : 
    latency(0.0)
{
}

void ModelFrame::clear()
{
  markerSets.clear();
  otherMarkers.clear();
  rigidBodies.clear();
}


ServerInfo::ServerInfo() :
    natNetVersion(0,0,0,0),
    serverVersion(0,0,0,0)
{

}


DataModel::DataModel() :
  hasValidServerInfo(false)
{

}

void DataModel::clear()
{
  dataFrame.clear();
}

void DataModel::setVersions(int* nver, int* sver)
{
  serverInfo.natNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
  serverInfo.serverVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
  hasValidServerInfo = true;
}

Version const& DataModel::getNatNetVersion() const
{
  return serverInfo.natNetVersion;
}

Version const& DataModel::getServerVersion() const
{
  return serverInfo.serverVersion;
}

}