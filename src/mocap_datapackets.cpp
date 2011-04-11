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
{
}

RigidBody::~RigidBody()
{
}

ModelDescription::ModelDescription()
{
}

ModelDescription::~ModelDescription()
{
}

ModelFrame::ModelFrame()
{
}

ModelFrame::~ModelFrame()
{
}

MoCapDataDescription::MoCapDataDescription()
{
}

MoCapDataDescription::~MoCapDataDescription()
{
  for (int i = 0; i < numDatasets; i++)
  {
    delete [] model[i].markerNames;
  }
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


MoCapDataFormat::MoCapDataFormat()
{
}

MoCapDataFormat::~MoCapDataFormat()
{
  for (int i = 0; i< numModels; i++)
  {
    delete[] model[i].markers;
    delete[] model[i].otherMarkers;
    for (int k = 0; k < model[i].numRigidBodies;k++)
    {
      delete[] model[i].rigidBodies[k].marker;
    }
    delete[] model[i].rigidBodies;
  }
  delete[] model;
}

 void MoCapDataFormat::parse(const char *packet, ushort payload)
{
  ushort parsingPosition = 4;	// bytes

  // parse frame number
  frameNumber = *((int*) &packet[parsingPosition]);
  parsingPosition += 4;

  // count number of datasets
  numModels = *((int*) &packet[parsingPosition]);
  parsingPosition += 4;

  model = new ModelFrame[numModels];
  for (int i = 0; i < numModels; i++)
  {
    for(ushort j = parsingPosition; j < payload;j++)
    {
      if (packet[j] == '\0' )
      {
	// read model's name
	model[i].name = &packet[parsingPosition];
	parsingPosition = j + 1;
	break;
      }
    }
    // read number of markers that belong to the model
    model[i].numMarkers = *((int*) &packet[parsingPosition]);
    parsingPosition += 4;
    model[i].markers = new Marker[model[i].numMarkers];
    for (int k = 0; k < model[i].numMarkers; k++)
    {
      // read marker positions
      model[i].markers[k].positionX = *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].markers[k].positionY = *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].markers[k].positionZ = *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
    }
		 
    // read number of 'other' markers (cf. NatNet specs)
    model[i].numOtherMarkers = *((int*) &packet[parsingPosition]);
    parsingPosition += 4;
    model[i].otherMarkers = new Marker[model[i].numMarkers];
    for (int l = 0; l < model[i].numOtherMarkers; l++)
    {
      // read positions of 'other' markers
      model[i].markers[l].positionX = *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].markers[l].positionY = *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].markers[l].positionZ = *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
    }
		 
    // read number of rigid bodies of the model
    model[i].numRigidBodies = *((int*) &packet[parsingPosition]);
    parsingPosition += 4;
    model[i].rigidBodies = new RigidBody [model[i].numRigidBodies];
    for (int m = 0; m < model[i].numRigidBodies; m++ )
    {
      // read id, position and orientation of each rigid body
      model[i].rigidBodies[m].ID =  *((int*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].positionX =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].positionY =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].positionZ =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].quaternionX =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].quaternionY =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].quaternionZ =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].quaternionW =  *((float*) &packet[parsingPosition]);
      parsingPosition += 4;

      // get number of markers per rigid body
      model[i].rigidBodies[m].NumberOfMarkers =  *((int*) &packet[parsingPosition]);
      parsingPosition += 4;
      model[i].rigidBodies[m].marker = new Marker [model[i].rigidBodies[m].NumberOfMarkers];
      for (int n = 0; n < model[i].rigidBodies[m].NumberOfMarkers; n++)
      {
	// get position for each marker
	model[i].rigidBodies[m].marker[n].positionX = *((float*) &packet[parsingPosition]);
	parsingPosition += 4;
	model[i].rigidBodies[m].marker[n].positionY = *((float*) &packet[parsingPosition]);
	parsingPosition += 4;
	model[i].rigidBodies[m].marker[n].positionZ = *((float*) &packet[parsingPosition]);
	parsingPosition += 4;
      }

    }
    // get latency
    model[i].latency = *((float*) &packet[parsingPosition]);
    parsingPosition += 4;
    parsingPosition += 4;		// skip terminating 0x00 00 00 00

  }

}


