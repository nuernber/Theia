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

#ifndef IMAGE_DESCRIPTOR_DESCRIPTOR_H_
#define IMAGE_DESCRIPTOR_DESCRIPTOR_H_

#include "image/keypoint_detector/keypoint.h"
#include <array>
#include <bitset>

namespace theia {
// NOTE: This enum must exactly match the descriptor.proto enum in order for the
// global proto conversion methods (below) to work.
enum DescriptorType {
  INVALID = -1,
  OTHER = 0,
  PATCH,
  SIFT,
  FREAK,
  BRISK
};

// Helper struct so that we can overload the operator[]. Bitsets return a
// special type of bitset::reference for non-const operator [], so it is not as
// simple to overload that operator from GenericDescriptor as just using the
// template type T for the lhs value.
template<typename T, std::size_t N>
struct TypeDeference {
  typedef typename std::array<T, N> Container;
  typedef typename std::array<T, N>::const_reference ConstRef;
  typedef typename std::array<T, N>::reference Ref;
};
// Specialization for bools and bitsets!
template<std::size_t N>
struct TypeDeference<bool, N>  {
  typedef typename std::bitset<N> Container;
  typedef bool ConstRef;
  typedef typename std::bitset<N>::reference Ref;
};

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
#define THEIA_INVALID_DESCRIPTOR_VAR -9999
template<class T, std::size_t N>
class GenericDescriptor {
 public:

  explicit GenericDescriptor(DescriptorType type)
      : descriptor_type_(type),
        x_(THEIA_INVALID_DESCRIPTOR_VAR),
        y_(THEIA_INVALID_DESCRIPTOR_VAR),
        strength_(THEIA_INVALID_DESCRIPTOR_VAR),
        scale_(THEIA_INVALID_DESCRIPTOR_VAR),
        orientation_(THEIA_INVALID_DESCRIPTOR_VAR) {}

  GenericDescriptor() : GenericDescriptor(DescriptorType::INVALID) {}

  ~GenericDescriptor() {}
  
  virtual inline void SetKeypoint(const Keypoint& keypoint) {
    x_ =keypoint.x();
    y_ = keypoint.y();
    strength_ = keypoint.strength();
    scale_ = keypoint.scale();
    orientation_ = keypoint.orientation();
  }

  // Accessor methods (implemented by Descriptor and BinaryDescriptor classes).
  typedef typename TypeDeference<T, N>::Ref TRef;
  typedef typename TypeDeference<T, N>::ConstRef TConstRef;
    typedef typename TypeDeference<T, N>::Container TContainer;
  virtual inline TRef operator[](std::size_t i) = 0;
  virtual inline TConstRef operator[](std::size_t i) const = 0;

  // Dimensionality of the descriptor.
  virtual inline std::size_t Dimensions() const { return N; }

  // Get the container (i.e. std::bitset or std::array) directly.
  virtual inline const TContainer& GetContainer() const = 0;
  
  // Descriptor type.
  inline DescriptorType descriptor_type() const { return descriptor_type_; }
  inline void set_descriptor_type(DescriptorType type) {
    descriptor_type_ = type;
  }

  // Variable x.
  inline double x() const { return x_; }
  inline void set_x(double x) { x_ = x; }

  // Variable y.
  inline double y() const { return y_; }
  inline void set_y(double y) { y_ = y; }

  // Optional variable strength.
  inline bool has_strength() const {
    return strength_ != THEIA_INVALID_DESCRIPTOR_VAR; }
  inline double strength() const { return strength_; }
  inline void set_strength(double strength) { strength_ = strength; }

  // Optional variable scale.
  inline bool has_scale() const {
    return scale_ != THEIA_INVALID_DESCRIPTOR_VAR;
  }
  inline double scale() const { return scale_; }
  inline void set_scale(double scale) { scale_ = scale; }

  // Optional variable orientation.
  inline bool has_orientation() const {
    return orientation_ != THEIA_INVALID_DESCRIPTOR_VAR; }
  inline double orientation() const { return orientation_; }
  inline void set_orientation(double orientation) {
    orientation_ = orientation; }

 protected:
  DescriptorType descriptor_type_;
  double x_;
  double y_;
  double strength_;
  double scale_;
  double orientation_;
};

// Class for **NON-BINARY** descriptors.
template<class T, std::size_t N>
class Descriptor : public GenericDescriptor<T, N> {
 public:
  Descriptor(DescriptorType type) : GenericDescriptor<T, N>(type) {}
  virtual ~Descriptor() {}
  
  typedef typename TypeDeference<T, N>::Ref TRef;
  typedef typename TypeDeference<T, N>::ConstRef TConstRef;
  typedef typename TypeDeference<T, N>::Container TContainer;
  virtual inline TRef operator[](std::size_t i) {return data_[i]; }
  virtual inline TConstRef operator[](std::size_t i) const { return data_[i]; }
  virtual inline const TContainer& GetContainer() const { return data_; }

  virtual inline T* Data() { return data_.data(); }
  virtual inline const T* Data() const { return data_.data(); }

 protected:
  std::array<T, N> data_;
};

// Class for binary descriptors.
template<std::size_t N>
class BinaryDescriptor : public GenericDescriptor<bool, N> {
 public:
  typedef typename std::bitset<N>::reference bitref;
  BinaryDescriptor(DescriptorType type) : GenericDescriptor<bool, N>(type) {}
  virtual ~BinaryDescriptor() {}

  typedef typename TypeDeference<bool, N>::Ref TRef;
  typedef typename TypeDeference<bool, N>::ConstRef TConstRef;
  typedef typename TypeDeference<bool, N>::Container TContainer;
  virtual inline TRef operator[](std::size_t i) { return binary_data_[i]; }
  virtual inline TConstRef operator[](std::size_t i) const {
    return binary_data_[i];
  }
  virtual inline const TContainer& GetContainer() const { return binary_data_; }
  
 protected:
  std::bitset<N> binary_data_;
};
}  // namespace theia
#endif  // IMAGE_DESCRIPTOR_DESCRIPTOR_H_
