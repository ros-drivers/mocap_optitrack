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

MoCapDataDescription::MoCapDataDescription()
  : model(0)
{
}

MoCapDataDescription::~MoCapDataDescription()
{
  delete [] model;
}

void MoCapDataDescription::parse(const char *packet, ushort payload)
{
  // tracks how many bytes from the incoming binary stream have already been processed
  ushort parsingPosition = 4;

  // get number of models
  numDatasets = *((int*) &packet[parsingPosition]);
  parsingPosition += 4;

  model = new ModelDescription [numDatasets];

  for (int i = 0 ;i < 1; i++)
  {
    parsingPosition += 4;					// Caution: this is missing in the NatNet specs
    for (int j = parsingPosition; j < payload; j++)
    {
      // find end of the model's name
      if (packet[j] == 0)
      {
	// read model's name
	model[i].name =&packet[parsingPosition];

	parsingPosition = j + 1;
	break;
      }
   }
	  
    // get model's number of markers
    model[i].numMarkers = *((int*) &packet[parsingPosition]);
    parsingPosition += 4;

    model[i].markerNames = new string[model[i].numMarkers];
    for (int k = 0; k < model[i].numMarkers; k++)
    {
      for (int l = parsingPosition; l < payload ; l++)
      {
	// find end of the marker's name
	if (packet[l] == 0)
	{
	  // read marker's name
	  model[i].markerNames[k] = &packet[parsingPosition];
	  parsingPosition = l + 1;
	  break;
	}
      }
    }

  }
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
  frameNumber = *((int*) packet);
  seek(sizeof(int));

  // count number of packetsets
  numModels = *((int*) packet);
  seek(sizeof(int));

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
    model[i].numMarkers = *((int*) packet);
    seek(sizeof(int));

    model[i].markers = new Marker[model[i].numMarkers];
    for (int k = 0; k < model[i].numMarkers; k++)
    {
      // read marker positions
      model[i].markers[k] = *((Marker*) packet);
      seek(sizeof(Marker));
    }
		 
    // read number of 'other' markers (cf. NatNet specs)
    model[i].numOtherMarkers = *((int*) packet);
    seek(sizeof(int));
    model[i].otherMarkers = new Marker[model[i].numOtherMarkers];
    for (int l = 0; l < model[i].numOtherMarkers; l++)
    {
      // read positions of 'other' markers
      model[i].markers[l] = *((Marker*) packet);
      seek(sizeof(Marker));
    }
		 
    // read number of rigid bodies of the model
    model[i].numRigidBodies = *((int*) packet);
    seek(sizeof(int));
    model[i].rigidBodies = new RigidBody [model[i].numRigidBodies];
    for (int m = 0; m < model[i].numRigidBodies; m++)
    {
      // read id, position and orientation of each rigid body
      model[i].rigidBodies[m].ID =  *((int*) packet);
      seek(sizeof(int));
      model[i].rigidBodies[m].pose.position.x =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.position.y =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.position.z =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.x =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.y =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.z =  *((float*) packet);
      seek(sizeof(float));
      model[i].rigidBodies[m].pose.orientation.w =  *((float*) packet);
      seek(sizeof(float));

      // get number of markers per rigid body
      model[i].rigidBodies[m].NumberOfMarkers =  *((int*) packet);
      seek(sizeof(int));
      model[i].rigidBodies[m].marker = new Marker [model[i].rigidBodies[m].NumberOfMarkers];
      for (int n = 0; n < model[i].rigidBodies[m].NumberOfMarkers; n++)
      {
        // get position for each marker
        model[i].markers[n] = *((Marker*) packet);
        seek(sizeof(Marker));
      }

    }
    // get latency
    model[i].latency = *((float*) packet);
    seek(sizeof(float));
  }

}


