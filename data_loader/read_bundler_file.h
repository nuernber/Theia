// Copyright (C) 2014 The Regents of the University of California (Regents).
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

#ifndef THEIA_DATA_LOADER_READ_BUNDLER_FILE_H_
#define THEIA_DATA_LOADER_READ_BUNDLER_FILE_H_

#include <Eigen/Core>
#include <string>
#include <theia/theia.h>
#include <vector>

// Loads the list of image names from the given bundler list file. Sets the
// focal length value to the given EXIF value if provided, and 0 otherwise.
bool ReadListsFile(const std::string& list_filename,
                   std::vector<std::string>* image_name,
                   std::vector<double>* exif_focal_length);

// Reads a SIFT key files as computed by Lowe's SIFT software:
// http://www.cs.ubc.ca/~lowe/keypoints/
bool ReadSiftKeyfile(const std::string& sift_key_file,
                     std::vector<Eigen::Vector2d>* feature_position,
                     std::vector<Eigen::VectorXf>* descriptor);

// Loads all information from a bundler file. The bundler file includes 3D
// points, camera poses, camera intrinsics, descriptors, and 2D-3D matches. As
// such, this method will load the SIFT descriptors from the corresponding SIFT
// key files.
//
// Input params are as follows:
//   bundler_file: the file output by bundler containing the 3D reconstruction,
//       camera poses, feature locations, and correspondences. Usually the file
//       is named bundle.out
//   sift_key_dir: The directory containing the sift keys used in the
//       reconstruction. Typically this is the base dir of the dataset. This
//       string should include a trailing slash.
//   image_list: A list of image filepaths that can be appeneded to sift_key_dir
//       to get the sift key file. The order should be the same as the order of
//       cameras in the bundler file. This is usually read from lists.txt
//   camera: A vector of theia::Camera objects that contain pose information,
//       descriptor information, and 2D-3D correspondences. The 3D point ids
//       stored correspond to the position in the world_points vector.
//   world_points: The 3D points from the reconstruction.
//   world_points_color: The RGB color of the world points (0 to 1 float).
bool ReadBundlerFile(const std::string& bundler_file,
                     const std::string& sift_key_dir,
                     const std::vector<std::string>& image_list,
                     std::vector<theia::Camera>* camera,
                     std::vector<Eigen::Vector3d>* world_points,
                     std::vector<Eigen::Vector3f>* world_points_color);

#endif  // THEIA_DATA_LOADER_READ_BUNDLER_FILE_H_
