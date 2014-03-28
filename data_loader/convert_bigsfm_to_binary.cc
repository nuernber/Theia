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

#include <Eigen/Core>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <stdint.h>
#include <theia/theia.h>

#include <fstream>
#include <string>
#include <vector>

DEFINE_string(bundler_file, "", "Filepath of the bundler.out file.");
DEFINE_string(list_file, "",
              "Filepath of the image lists used in the reconstruction.");

DEFINE_string(
    sift_key_dir, "",
    "Base directory for the sift key files, as specified by the list file.");

DEFINE_string(
    output_binary, "",
    "Filepath for the binary file to be written. This file may be massive!");

// Writes reconstruction data from a binary file. The file has the following
// format:
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
bool OutputBinaryFile(const std::vector<theia::Camera>& cameras,
                      const std::vector<Eigen::Vector3d>& world_points,
                      const std::vector<Eigen::Vector3f>& world_points_color,
                      const std::string output_binary) {
  std::ofstream ofs(output_binary.c_str(), std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    LOG(ERROR) << "Could not write the bundler file to " << output_binary;
    return false;
  }

  // Number of 3d points.
  const uint64_t num_points = static_cast<uint64_t>(world_points.size());
  ofs.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

  // Write each 3D point and its color.
  for (int i = 0; i < num_points; i++) {
    const Eigen::Vector3d& world_point = world_points[i];
    ofs.write(reinterpret_cast<const char*>(world_point.data()),
              sizeof(world_point));
    const Eigen::Vector3f& world_point_color = world_points_color[i];
    ofs.write(reinterpret_cast<const char*>(world_point_color.data()),
              sizeof(world_point_color));
  }

  // Number of cameras.
  const uint64_t num_cameras = static_cast<uint64_t>(cameras.size());
  ofs.write(reinterpret_cast<const char*>(&num_cameras), sizeof(num_cameras));

  // Write out each camera.
  for (int cam_index = 0; cam_index < cameras.size(); cam_index++) {
    const theia::Camera& camera = cameras[cam_index];
    // Output focal length.
    const double focal_length = camera.pose_.focal_length();
    ofs.write(reinterpret_cast<const char*>(&focal_length),
              sizeof(focal_length));

    // Output 2 radial distortion params.
    double k1, k2, k3, k4;
    camera.pose_.radial_distortion(&k1, &k2, &k3, &k4);
    ofs.write(reinterpret_cast<const char*>(&k1), sizeof(k1));
    ofs.write(reinterpret_cast<const char*>(&k2), sizeof(k2));

    // Output Rotation. Eigen stores matrices in column-major while Bundler
    // prefers row-major so we take the transpose of the rotation matrix so that
    // the orderin is consistent (i.e., the output will be row-major).
    const Eigen::Matrix3d rotation_row_major =
        camera.pose_.rotation_matrix().transpose();
    ofs.write(reinterpret_cast<const char*>(rotation_row_major.data()),
              sizeof(rotation_row_major));

    // Output translation.
    const Eigen::Vector3d translation = camera.pose_.translation();
    ofs.write(reinterpret_cast<const char*>(translation.data()),
              sizeof(translation));

    // Check features are correct sizes.
    CHECK_EQ(camera.feature_position_2D_distorted_.size(),
             camera.feature_position_2D_.size());
    CHECK_EQ(camera.feature_position_2D_distorted_.size(),
             camera.feature_3D_ids_.size());
    CHECK_EQ(camera.feature_position_2D_distorted_.size(),
             camera.descriptors_.size());

    // Output number of features.
    const uint64_t num_features =
        static_cast<uint64_t>(camera.feature_position_2D_distorted_.size());
    ofs.write(reinterpret_cast<const char*>(&num_features),
              sizeof(num_features));
    // For each feature.
    for (int i = 0; i < num_features; i++) {
      // Output measured (i.e., distorted) position
      ofs.write(reinterpret_cast<const char*>(
                    camera.feature_position_2D_distorted_[i].data()),
                sizeof(camera.feature_position_2D_distorted_[i]));

      // Corrected position.
      ofs.write(
          reinterpret_cast<const char*>(camera.feature_position_2D_[i].data()),
          sizeof(camera.feature_position_2D_[i]));

      // 3D point id.
      const uint64_t feature_3D_id =
          static_cast<uint64_t>(camera.feature_3D_ids_[i]);
      ofs.write(reinterpret_cast<const char*>(&feature_3D_id),
                sizeof(feature_3D_id));

      // SIFT descriptor.
      ofs.write(reinterpret_cast<const char*>(camera.descriptors_[i].data()),
                sizeof(camera.descriptors_[i]));
    }

    if ((cam_index + 1) % 100 == 0 || cam_index == num_cameras - 1) {
      std::cout << "\r Writing parameters for camera " << cam_index + 1 << " / "
                << num_cameras << std::flush;
    }
  }
  std::cout << std::endl;
  ofs.close();
  return true;
}

// Given the view list from the Bundler file, this function will load the sift
// descriptors from the key files and store them with the cameras.
bool LoadSiftCorrespondences(
    const std::vector<std::string>& image_list, const std::string& sift_key_dir,
    const std::vector<theia::BundlerViewList>& view_list,
    std::vector<theia::Camera>* camera) {
  // First, load all SIFT descriptors into memory.
  for (int i = 0; i < image_list.size(); i++) {
    // Get image file and trim the .jpg off
    std::string sift_key_file = image_list[i];
    static const std::string extension = ".jpg";
    const size_t ext_loc = sift_key_file.rfind(extension);
    if (ext_loc != std::string::npos) {
      sift_key_file = sift_key_dir + sift_key_file.substr(0, ext_loc) + ".key";
    } else {
      LOG(ERROR) << "could not parse the image filename properly!";
      return false;
    }

    CHECK(theia::ReadSiftKeyfile(
        sift_key_file, &(camera->at(i).feature_position_2D_distorted_),
        &(camera->at(i).descriptors_)));

    // Set the size of the 3d position IDs to equal the 2d position size.
    camera->at(i).feature_3D_ids_
        .resize(camera->at(i).feature_position_2D_distorted_.size());

    // Undistort the image points and add them to the camera.
    camera->at(i).pose_
        .UndistortImagePoint(camera->at(i).feature_position_2D_distorted_,
                             &(camera->at(i).feature_position_2D_));

    // Update progress
    if ((i + 1) % 100 == 0 || i == image_list.size() - 1) {
      std::cout << "\r Loading sift features for camera " << i + 1 << " / "
                << image_list.size() << std::flush;
    }
  }
  std::cout << std::endl;

  // Go through the view list and add the 2D-3D correspondences.
  for (int i = 0; i < view_list.size(); i++) {
    for (int j = 0; j < view_list[i].size(); j++) {
      const int camera_index = view_list[i][j].camera_index;
      const int sift_key_index = view_list[i][j].sift_key_index;

      CHECK_LT(camera_index, camera->size());
      CHECK_LT(sift_key_index,
               camera->at(camera_index).feature_position_2D_.size())
          << "Sift key failure! camera(" << camera_index
          << ") = " << image_list[camera_index];

        camera->at(camera_index).feature_3D_ids_[sift_key_index] = i;
    }
    // Update progress
    if ((i + 1) % 100 == 0 || i == view_list.size() - 1) {
      std::cout << "\r Loading correspondences for point " << i + 1 << " / "
                << view_list.size() << std::flush;
    }
  }
  std::cout << std::endl;
  return true;
}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Read lists file.
  std::vector<std::string> image_name;
  std::vector<double> exif_focal_length;
  CHECK(theia::ReadListsFile(FLAGS_list_file, &image_name, &exif_focal_length));

  // Read bundler file.
  std::vector<theia::Camera> cameras;
  std::vector<Eigen::Vector3d> world_points;
  std::vector<Eigen::Vector3f> world_points_color;
  std::vector<theia::BundlerViewList> view_list;
  CHECK(theia::ReadBundlerFile(FLAGS_bundler_file, &cameras, &world_points,
                               &world_points_color, &view_list));

  // Load the SIFT descriptors into the cameras.
  CHECK(LoadSiftCorrespondences(image_name, FLAGS_sift_key_dir, view_list,
                                &cameras));

  // Output as a binary file.
  CHECK(OutputBinaryFile(cameras, world_points, world_points_color,
                         FLAGS_output_binary));

  return 0;
}
