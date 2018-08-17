/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, University of Bonn, Computer Science Institute VI
 *  Author: Kathrin Gräve, 01/2011
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>

#ifndef __MOCAP_DATAPACKETS_H__
#define __MOCAP_DATAPACKETS_H__

#include <sys/types.h>
#include <iostream>
#include <string>


#include <mocap_optitrack/version.h>

using namespace std;

/// \brief Parser for a NatNet data frame packet
class NatNetParser
{
  public:
    NatNetParser();
    ~NatNetParser();

    /// \brief Parses a NatNet data frame packet as it is streamed by the Arena software according to the descriptions in the NatNet SDK v1.4
    void parse(const char* msg_buffer, int numBytes);

    ModelFrame model;

    Version natNetVersion;
    Version serverVersion;
    bool haveVersion;

  private:
    void setVersion(int nver[4], int sver[4])
    {
      natNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
      serverVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
    }

    void decodeMarkerID(int sourceID, int* pOutEntityID, int* pOutMemberID);
    bool decodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);
    bool timecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize);

    void parseRigidBody(RigidBody&);
    void parseMocapDataFrame();
};

#endif  /*__MOCAP_DATAPACKETS_H__*/
