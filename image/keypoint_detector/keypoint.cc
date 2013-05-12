// Copyright (C) 2013 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "image/keypoint_detector/keypoint.h"

#include <vector>

#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/keypoint_detector/keypoint.pb.h"
#endif

namespace theia {
// Global methods to convert between protocol buffers.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
bool ProtoToKeypoint(const KeypointsProto& proto,
                     std::vector<Keypoint*>* keypoints) {
  for (const KeypointProto& proto_keypoint : proto.keypoint()) {
    Keypoint* keypoint = new Keypoint(proto_keypoint.location().x(),
                                      proto_keypoint.location().y(),
                                      static_cast<Keypoint::KeypointType>(
                                          proto_keypoint.keypoint_detector()));
    // TODO(cmsweeney): Check this... I'm not sure that the static cast is
    // stable.
    keypoint->set_keypoint_type(static_cast<Keypoint::KeypointType>(
        proto_keypoint.keypoint_detector()));

    // Set the strength.
    if (proto_keypoint.has_strength())
      keypoint->set_strength(proto_keypoint.strength());
    // Set the scale.
    if (proto_keypoint.has_scale())
      keypoint->set_scale(proto_keypoint.scale());
    // Set the orientation.
    if (proto_keypoint.has_orientation())
      keypoint->set_orientation(proto_keypoint.orientation());
    keypoints->push_back(keypoint);
  }
  return true;
}

bool KeypointToProto(const std::vector<Keypoint*>& keypoints,
                     KeypointsProto* proto) {
  for (const Keypoint* keypoint : keypoints) {
    KeypointProto* keypoint_proto = proto->add_keypoint();
    // Set the location.
    KeypointProto_Location* keypoint_location =
        keypoint_proto->mutable_location();
    keypoint_location->set_x(keypoint->x());
    keypoint_location->set_y(keypoint->y());

    // Set the type.
    keypoint_proto->set_keypoint_detector(
        static_cast<KeypointProto_KeypointType>(keypoint->keypoint_type()));

    // Set the strength.
    if (keypoint->has_strength())
      keypoint_proto->set_strength(keypoint->strength());
    // Set the scale.
    if (keypoint->has_scale())
      keypoint_proto->set_scale(keypoint->scale());
    // Set the orientation.
    if (keypoint->has_orientation())
      keypoint_proto->set_orientation(keypoint->orientation());
  }
  return true;
}
#endif
}  // namespace theia
