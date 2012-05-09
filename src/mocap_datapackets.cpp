#include "mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <iostream>
using namespace std;


Marker::Marker()
{
}

Marker::~Marker()
{
}


RigidBody::RigidBody() 
  : pose(), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

ModelDescription::ModelDescription()
  : markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markers(0), otherMarkers(0), rigidBodies(0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markers;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length) 
  : packet(packet), length(length)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
  delete[] model;
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse()
{
  seek(4);
  
  // parse frame number
  read_and_seek(frameNumber);

  // count number of packetsets
  read_and_seek(numModels);

  model = new ModelFrame[numModels];
  for (int i = 0; i < numModels; i++)
  {
    while (*packet != '\0')
    {
      model[i].name.push_back(*packet);
      seek(1);
    }
    seek(1);

    // read number of markers that belong to the model
    read_and_seek(model[i].numMarkers);
      printf("NumMarkers: %d\n", model[i].numMarkers);

    model[i].markers = new Marker[model[i].numMarkers];
    for (int k = 0; k < model[i].numMarkers; k++)
    {
      // read marker positions
      read_and_seek(model[i].markers[k]);
    }
 
    // read number of 'other' markers (cf. NatNet specs)
    read_and_seek(model[i].numOtherMarkers);
    model[i].otherMarkers = new Marker[model[i].numOtherMarkers];
    for (int l = 0; l < model[i].numOtherMarkers; l++)
    {
      // read positions of 'other' markers
      read_and_seek(model[i].otherMarkers[l]);
    }
 
    // read number of rigid bodies of the model
    read_and_seek(model[i].numRigidBodies);
      printf("NumRigidBodies: %d\n", model[i].numRigidBodies);

    model[i].rigidBodies = new RigidBody[model[i].numRigidBodies];
    for (int m = 0; m < model[i].numRigidBodies; m++)
    {
      // read id, position and orientation of each rigid body
      read_and_seek(model[i].rigidBodies[m].ID);
      read_and_seek(model[i].rigidBodies[m].pose.position.x);
      read_and_seek(model[i].rigidBodies[m].pose.position.y);
      read_and_seek(model[i].rigidBodies[m].pose.position.z);
      read_and_seek(model[i].rigidBodies[m].pose.orientation.x);
      read_and_seek(model[i].rigidBodies[m].pose.orientation.y);
      read_and_seek(model[i].rigidBodies[m].pose.orientation.z);
      read_and_seek(model[i].rigidBodies[m].pose.orientation.w);

      // get number of markers per rigid body
      read_and_seek(model[i].rigidBodies[m].NumberOfMarkers);
      printf("NumRBMarkers: %d\n", model[i].rigidBodies[m].NumberOfMarkers);
      if (model[i].rigidBodies[m].NumberOfMarkers > 0 && 0)
      {
          model[i].rigidBodies[m].marker = new Marker [model[i].rigidBodies[m].NumberOfMarkers];
          for (int n = 0; n < model[i].rigidBodies[m].NumberOfMarkers; n++)
          {
            // get position for each marker
            read_and_seek(model[i].rigidBodies[m].marker[n]);
          }
      }

    }
    // get latency
    read_and_seek(model[i].latency);
  }

}


