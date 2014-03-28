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

#include "theia/data_loader/read_bigsfm_binary_file.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

namespace theia {

// Loads Big SfM reconstruction data from a binary file. The original Big SfM
// datasets can be found here: http://www.cs.cornell.edu/projects/bigsfm/#data
//
// The file has the following format:
//
// number of 3D points (uint64_t)
// for each 3D point: x y z (doubles) r g b (floats)
// number of cameras (uint64_t)
// for each camera:
//   focal length, two radial distortion parameters (all double)
//   rotation matrix and translation vector (following Bundler's convention, all
//     represented as doubles, rotation is given in row-major order)
//   number of 2D features (uint64_t)
//   for each feature:
//     x_m y_m x_u y_u (2D image positions for the original image feature
//       measurement and the corresponding undistorted feature position, both
//       relative to the center of the image ((width-1)/2, (height-1)/2), all
//       represented as doubles)
//     3D point id (uint64_t, the id of the corresponding 3D point in the list
//       of 3D points loaded before)
//     128-float SIFT descriptor of the feature.
bool ReadBigSfMBinary(const std::string& binary_file,
                      std::vector<theia::Camera>* cameras,
                      std::vector<Eigen::Vector3d>* world_points,
                      std::vector<Eigen::Vector3f>* world_points_color) {
  std::ifstream ifs(binary_file.c_str(), std::ios::in | std::ios::binary);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Could not read the binary big sfm file from "
               << binary_file;
    return false;
  }

  // Number of 3d points.
  uint64_t num_points;
  ifs.read(reinterpret_cast<char*>(&num_points), sizeof(num_points));

  // Resize world_points appropriately.
  world_points->clear();
  world_points->resize(num_points);
  world_points_color->clear();
  world_points_color->resize(num_points);
  // Read each 3D point.
  for (int i = 0; i < num_points; i++) {
    ifs.read(reinterpret_cast<char*>(world_points->at(i).data()),
             sizeof(world_points->at(i)));
    ifs.read(reinterpret_cast<char*>(world_points_color->at(i).data()),
             sizeof(world_points_color->at(i)));
  }

  // Number of cameras.
  uint64_t num_cameras;
  ifs.read(reinterpret_cast<char*>(&num_cameras), sizeof(num_cameras));

  // Resize cameras appropriately.
  cameras->clear();
  cameras->resize(num_cameras);

  // Read out each camera.
  for (int cam_index = 0; cam_index < num_cameras; cam_index++) {
    theia::Camera& camera = cameras->at(cam_index);
    // Read focal length.
    double focal_length = camera.pose_.focal_length();
    ifs.read(reinterpret_cast<char*>(&focal_length),
             sizeof(focal_length));

    const Eigen::Matrix3d calibration_matrix =
        Eigen::DiagonalMatrix<double, 3>(focal_length, focal_length, 1.0);

    // Read the 2 radial distortion params.
    double k1, k2;
    ifs.read(reinterpret_cast<char*>(&k1), sizeof(k1));
    ifs.read(reinterpret_cast<char*>(&k2), sizeof(k2));

    // Read Rotation. Eigen stores matrices in column-major while Bundler
    // prefers row-major so we take the transpose of the rotation matrix so that
    // the orderin is consistent (i.e., the output will be row-major).
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation_row_major;
    ifs.read(reinterpret_cast<char*>(rotation_row_major.data()),
              sizeof(rotation_row_major));

    // Output translation.
    Eigen::Vector3d translation;
    ifs.read(reinterpret_cast<char*>(translation.data()),
              sizeof(translation));

    // Initialize the pose.
    camera.pose_.InitializePose(rotation_row_major, translation,
                                calibration_matrix, k1, k2, 0.0, 0.0);

    // Read number of features.
    uint64_t num_features;
    ifs.read(reinterpret_cast<char*>(&num_features),
              sizeof(num_features));

    // Resize feature containers appropriately.
    camera.feature_position_2D_distorted_
        .resize(num_features, Eigen::Vector2d::Zero());
    camera.feature_position_2D_.resize(num_features, Eigen::Vector2d::Zero());
    camera.feature_3D_ids_.resize(num_features);
    Eigen::VectorXf zero_desc(128);
    zero_desc.setZero();
    camera.descriptors_.resize(num_features, zero_desc);

    // For each feature.
    for (int i = 0; i < num_features; i++) {
      // Output measured (i.e., distorted) position
      ifs.read(reinterpret_cast<char*>(
                    camera.feature_position_2D_distorted_[i].data()),
                sizeof(camera.feature_position_2D_distorted_[i]));

      // Corrected position.
      ifs.read(
          reinterpret_cast<char*>(camera.feature_position_2D_[i].data()),
          sizeof(camera.feature_position_2D_[i]));

      // 3D point id.
      uint64_t feature_3D_id;
      ifs.read(reinterpret_cast<char*>(&feature_3D_id),
                sizeof(feature_3D_id));
      camera.feature_3D_ids_[i] = feature_3D_id;

      // SIFT descriptor.
      ifs.read(reinterpret_cast<char*>(camera.descriptors_[i].data()),
                sizeof(camera.descriptors_[i]));
    }

    if ((cam_index + 1) % 100 == 0 || cam_index == num_cameras - 1) {
      std::cout << "\r Reading parameters for camera " << cam_index + 1 << " / "
                << num_cameras << std::flush;
    }
  }
  std::cout << std::endl;
  ifs.close();
  return true;
}

}  // namespace theia
