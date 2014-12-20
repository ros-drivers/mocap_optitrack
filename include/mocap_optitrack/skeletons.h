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
///
/// This class contains arrays of names corresponding to the rigid body ids
/// of the three skeletons that are predefined by the Arena software 

/// \brief Rigid body names for the skeleton with toes
char const * SKELETON_WITH_TOES[] =
{
  "",
  "hip",
  "ab",
  "chest",
  "neck",
  "head",
  "head_end",
  "left_shoulder",
  "left_upper_arm",
  "left_fore_arm",
  "left_hand",
  "left_hand_end",
  "right_shoulder",
  "right_upper_arm",
  "right_fore_arm",
  "right_hand",
  "right_hand_end",
  "left_thigh",
  "left_shin",
  "left_foot",
  "left_toe",
  "left_toe_end",
  "right_thigh",
  "right_shin",
  "right_foot",
  "right_toe",
  "right_toe_end"
};

/// Rigid body names for the skeleton without toes
char const * SKELETON_WITHOUT_TOES[] =
{
  "",
  "hip",
  "ab",
  "chest",
  "neck",
  "head",
  "head_end",
  "left_shoulder",
  "left_upper_arm",
  "left_fore_arm",
  "left_hand",
  "left_hand_end",
  "right_shoulder",
  "right_upper_arm",
  "right_fore_arm",
  "right_hand",
  "right_hand_end",
  "left_thigh",
  "left_shin",
  "left_foot",
  "left_foot_end",
  "right_thigh",
  "right_shin",
  "right_foot",
  "right_foot_end"
};

/// Rigid body name for a custom rigid body
char const * OBJECT[] =
{
  "",
  "object"
};
