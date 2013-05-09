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

#ifndef IMAGE_DESCRIPTOR_DESCRIPTOR_H_
#define IMAGE_DESCRIPTOR_DESCRIPTOR_H_

#include <array>
#include <bitset>

namespace theia {
// The descriptor will hold N dimensions of type T (e.g. SIFT is 128 dims of
// float). Most new descriptors should derive from either the Descriptor or
// BinaryDescriptor class. Those classes have been specifically optimized for
// fixed-size descriptors of various data types (float, int, binary,
// etc.).
//
// Also, note that templating this class will ensure that only descriptors with
// the same template parameters can be compared (for matching, etc.). This is
// opposed to allocating data pointers to arrays at runtime, which provides no
// guarantees that two classes will have the same structure!
template<class T, std::size_t N>
class GenericDescriptor {
 public:
  virtual ~GenericDescriptor() {}

  // Accessor methods (implemented by Descriptor and BinaryDescriptor classes).
  virtual inline T& operator[](std::size_t i) = 0;
  virtual inline T operator[](std::size_t i) const = 0;

  // Dimensionality of the descriptor.
  virtual inline std::size_t Dimensions() const { return N; }
};

// Class for **NON-BINARY** descriptors.
template<class T, std::size_t N>
class Descriptor : public GenericDescriptor<T, N> {
 public:
  virtual ~Descriptor() {}
  virtual inline T& operator[](std::size_t i) { return data_[i]; }
  virtual inline T operator[](std::size_t i) const { return data_[i]; }
  virtual inline T* Data() { return data_.data(); }

 protected:
  std::array<T, N> data_;
};

// Class for binary descriptors.
template<std::size_t N>
class BinaryDescriptor : public GenericDescriptor<bool, N> {
 public:
  virtual ~BinaryDescriptor() {}
  virtual inline bool& operator[](std::size_t i) { return binary_data_[i]; }
  virtual inline bool operator[](std::size_t i) const {return binary_data_[i]; }

  // TODO(cmsweeney): Should we add a data() method that returns the string?

 protected:
  std::bitset<N> binary_data_;
};

}  // namespace theia

#endif  // IMAGE_DESCRIPTOR_DESCRIPTOR_H_
