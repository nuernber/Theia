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
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef IMAGE_KEYPOINT_DETECTOR_KEYPOINT_DETECTOR_H_
#define IMAGE_KEYPOINT_DETECTOR_KEYPOINT_DETECTOR_H_

#include <vector>

namespace theia {
template<class T> class Image;
typedef Image<float> GrayImage;
class Keypoint;
class KeypointsProto;

// A pure virtual class for keypoint detectors. We assume that the keypoint
// detectors only use grayimages for now.
class KeypointDetector {
 public:
  KeypointDetector() {}
  virtual ~KeypointDetector() {}

  // Use this method to initialize any internals. Only use the constructor for
  // basic operations since the debug trace is limited for errors in the
  // constructor.
  virtual bool Initialize() { return true; }

  // Detect keypoints using the desired method (and fill Keypoint with the
  // desired Keypoint, which may be a subclass!). This means the caller will
  // have to cast the pointers from the vector appropriately in order to get
  // access to the derived class object (e.g. cast keypoints[i] as a
  // FastKeypoint*). Caller owns the data returned.
  virtual bool DetectKeypoints(const GrayImage& image,
                               std::vector<Keypoint*>* keypoints) = 0;

  // Methods to load/store keypoints in protocol buffers. Each derived class
  // should implement these methods (if desired) and load/store all appropriate
  // fields in the protocol buffer. This is kind of a sucky paradigm since these
  // methods act more like static methods, but it is the best way to make sure
  // these methods are paired to the keypoints.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
  virtual bool ProtoToKeypoint(const KeypointsProto& proto,
                               std::vector<Keypoint*>* keypoints) const = 0;
  virtual bool KeypointToProto(const std::vector<Keypoint*>& keypoints,
                               KeypointsProto* proto) const = 0;
#endif
};
}  // namespace theia

#endif  // IMAGE_KEYPOINT_DETECTOR_KEYPOINT_DETECTOR_H_
