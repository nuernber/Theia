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

#include "read_bundler_file.h"

#include <Eigen/Core>
#include <theia/theia.h>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

// Description of the list files from the Big SfM website:
// http://www.cs.cornell.edu/projects/p2f/README_Dubrovnik6K.txt
//
// A. List files (list.db.txt, list.query.txt).
//      List files specify filenames to images in jpg format, one per
//      line (keep in mind that the actual jpg files are not distributed
//      unless requested).  In addition, if the focal length of the image
//      has been estimated from Exif tags, then that is also included.
//
//      Images without known focal length information are specified with
//      a line with a single field, the image name.  Example:
//        query/10970812@N05_2553027508.jpg
//
//      Images with known focal length information are specified with a
//      line with three fields: the image name, a zero, and the Exif
//      focal length.  (The second field is always zero but may change in
//      future datasets.)  Example:
//        query/11289373@N03_2733280477.jpg 0 1280.00000
//
// NOTE: We set the exif focal length to zero if it is not available (since 0 is
// never a valid focal length).
bool ReadListsFile(const std::string& list_filename,
                   std::vector<std::string>* image_name,
                   std::vector<double>* exif_focal_length) {
  std::ifstream ifs(list_filename.c_str(), std::ios::in);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Cannot read the list file from " << list_filename;
    return false;
  }

  const char space = static_cast<char>(' ');
  while (!ifs.eof()) {
    // Read in the filename.
    std::string filename;
    ifs >> filename;
    if (filename.length() == 0) {
      break;
    }

    image_name->push_back(filename);

    // Check to see if the exif focal length is given.
    double focal_length = 0;
    if (ifs.peek() == space) {
      int temp;
      ifs >> temp;
      ifs >> focal_length;
    }
    exif_focal_length->push_back(focal_length);
  }
  return true;
}

// The sift key file has the following format:
//
// number_of_keypoints sift_descriptor_dimensions (both as ints)
// for each descriptor:
//   row col scale orientation (all as floats)
//   128 ints describing sift descriptor. Normalizing this 128-vector to unit
//     length will yield the true sift descriptor.
// NOTE: We use getline and strtof which should be much faster than letting the
// stream parse the string with operator >>.
bool ReadSiftKeyfile(const std::string& sift_key_file,
                     std::vector<Eigen::Vector2d>* feature_position,
                     std::vector<Eigen::VectorXf>* descriptor) {
  std::ifstream ifs(sift_key_file.c_str(), std::ios::in);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Cannot read the sift key file from " << sift_key_file;
    return false;
  }

  std::string header_string;
  std::getline(ifs, header_string);
  const char* p = header_string.c_str();
  char* p2;
  int num_descriptors = strtol(p, &p2, 10);
  p = p2;
  int num_dimensions = strtol(p, &p2, 10);
  CHECK_EQ(num_dimensions, 128);

  // Read each descriptor one by one.
  feature_position->reserve(num_descriptors);
  descriptor->reserve(num_descriptors);
  for (int i = 0; i < num_descriptors; i++) {
    // Read in row col scale.
    Eigen::Vector2d feature_pos;
    std::string feature_header;
    std::getline(ifs, feature_header);
    p = feature_header.c_str();
    feature_pos.x() = strtof(p, &p2);
    p = p2;
    feature_pos.y() = strtof(p, &p2);
    feature_position->push_back(feature_pos);

    // Read in descriptor values.
    Eigen::VectorXf sift_descriptor(128);
    int dimension = 0;
    while (dimension < num_dimensions) {
      std::string descriptor_string;
      std::getline(ifs, descriptor_string);
      p = descriptor_string.c_str();
      float val;
      while (val = std::strtof(p, &p2), p != p2) {
        sift_descriptor(dimension) = val;
        p = p2;
        dimension++;
      }
    }
    sift_descriptor.normalize();
    descriptor->push_back(sift_descriptor);
  }
  ifs.close();

  return true;
}

// The bundle files contain the estimated scene and camera geometry have the
// following format:
//     # Bundle file v0.3
//     <num_cameras> <num_points>   [two integers]
//     <camera1>
//     <camera2>
//        ...
//     <cameraN>
//     <point1>
//     <point2>
//        ...
//     <pointM>
// Each camera entry <cameraI> contains the estimated camera intrinsics and
// extrinsics, and has the form:
//     <f> <k1> <k2>   [the focal length, followed by two radial distortion
//                      coeffs]
//     <R>             [a 3x3 matrix representing the camera rotation]
//     <t>             [a 3-vector describing the camera translation]
// The cameras are specified in the order they appear in the list of images.
//
// Each point entry has the form:
//     <position>      [a 3-vector describing the 3D position of the point]
//     <color>         [a 3-vector describing the RGB color of the point]
//     <view list>     [a list of views the point is visible in]
//
// The view list begins with the length of the list (i.e., the number of cameras
// the point is visible in). The list is then given as a list of quadruplets
// <camera> <key> <x> <y>, where <camera> is a camera index, <key> the index of
// the SIFT keypoint where the point was detected in that camera, and <x> and
// <y> are the detected positions of that keypoint. Both indices are 0-based
// (e.g., if camera 0 appears in the list, this corresponds to the first camera
// in the scene file and the first image in "list.txt"). The pixel positions are
// floating point numbers in a coordinate system where the origin is the center
// of the image, the x-axis increases to the right, and the y-axis increases
// towards the top of the image. Thus, (-w/2, -h/2) is the lower-left corner of
// the image, and (w/2, h/2) is the top-right corner (where w and h are the
// width and height of the image).
bool ReadBundlerFile(const std::string& bundler_file,
                     const std::string& sift_key_dir,
                     const std::vector<std::string>& image_list,
                     std::vector<theia::Camera>* camera,
                     std::vector<Eigen::Vector3d>* world_points,
                     std::vector<Eigen::Vector3f>* world_points_color) {
  // Read in num cameras, num points.
  std::ifstream ifs(bundler_file.c_str(), std::ios::in);
  if (!ifs.is_open()) {
    LOG(ERROR) << "Cannot read the bundler file from " << bundler_file;
    return false;
  }

  std::string header_string;
  // There is one line of filler, so skip that line!
  std::getline(ifs, header_string);

  std::getline(ifs, header_string);
  const char* p = header_string.c_str();
  char* p2;
  const int num_cameras = strtol(p, &p2, 10);
  p = p2;
  const int num_points = strtol(p, &p2, 10);

  CHECK_LE(num_cameras, image_list.size());

  // Allocate the proper number of cameras.
  camera->clear();
  camera->resize(num_cameras);

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
    if ((i + 1) % 100 == 0 || i == image_list.size() - 1) {
      std::cout << "\r Loading sift features for camera " << i + 1 << " / "
                << image_list.size() << std::flush;
    }

    CHECK(ReadSiftKeyfile(sift_key_file,
                          &(camera->at(i).feature_position_2D_distorted_),
                          &(camera->at(i).descriptors_)));

    // Set the size of the 3d position IDs to equal the 2d position size.
    camera->at(i).feature_3D_ids_
        .resize(camera->at(i).feature_position_2D_distorted_.size());
    // Resize the undistorted feature positions to equal the distorted position
    // size.
    camera->at(i).feature_position_2D_
        .resize(camera->at(i).feature_position_2D_distorted_.size());
  }
  std::cout << std::endl;

  // Read in the camera params.
  for (int i = 0; i < num_cameras; i++) {
    // Read in focal length, radial distortion.
    std::string internal_params;
    std::getline(ifs, internal_params);
    p = internal_params.c_str();
    const double focal_length = strtod(p, &p2);
    p = p2;
    const double k1 = strtod(p, &p2);
    p = p2;
    const double k2 = strtod(p, &p2);
    p = p2;


    const Eigen::Matrix3d calibration_matrix =
        Eigen::DiagonalMatrix<double, 3>(focal_length, focal_length, 1.0);
    // Read in rotation (row-major).
    Eigen::Matrix3d rotation;
    for (int r = 0; r < 3; r++) {
      std::string rotation_row;
      std::getline(ifs, rotation_row);
      p = rotation_row.c_str();

      for (int c = 0; c < 3; c++) {
        rotation(r, c) = strtod(p, &p2);
        p = p2;
      }
    }

    std::string translation_string;
    std::getline(ifs, translation_string);
    p = translation_string.c_str();
    Eigen::Vector3d translation;
    for (int j = 0; j < 3; j++) {
      translation(j) = strtod(p, &p2);
      p = p2;
    }
    camera->at(i).pose_.InitializePose(rotation, translation,
                                       calibration_matrix, k1, k2, 0.0, 0.0);

    if ((i + 1) % 100 == 0 || i == num_cameras - 1) {
      std::cout << "\r Loading parameters for camera " << i + 1 << " / "
                << num_cameras << std::flush;
    }
  }
  std::cout << std::endl;

  // Read in each 3D point and correspondences.
  world_points->clear();
  world_points->resize(num_points);
  world_points_color->clear();
  world_points_color->resize(num_points);
  for (int i = 0; i < num_points; i++) {
    // Read position.
    std::string position;
    std::getline(ifs, position);
    p = position.c_str();
    for (int j = 0; j < 3; j++) {
      world_points->at(i)(j) = strtod(p, &p2);
      p = p2;
    }

    // Read color.
    std::string color;
    std::getline(ifs, color);
    p = color.c_str();
    for (int j = 0; j < 3; j++) {
      world_points_color->at(i)(j) =
          static_cast<float>(strtol(p, &p2, 10)) / 255.0;
      p = p2;
    }


    // Read viewlist.
    std::string view_list;
    std::getline(ifs, view_list);
    p = view_list.c_str();
    const int num_views = strtol(p, &p2, 10);
    p = p2;
    for (int j = 0; j < num_views; j++) {
      // Camera key x y
      const int camera_index = strtol(p, &p2, 10);
      p = p2;
      const int sift_key_index = strtol(p, &p2, 10);
      p = p2;
      const float x_pos = strtof(p, &p2);
      p = p2;
      const float y_pos = strtof(p, &p2);
      p = p2;
      CHECK_EQ(x_pos, 0.0);
      CHECK_EQ(y_pos, 0.0);

      CHECK_LT(camera_index, camera->size()) << "string = " << view_list;
      CHECK_LT(sift_key_index,
               camera->at(camera_index).feature_position_2D_.size())
          << "Sift key failure! camera(" << camera_index
          << ") = " << image_list[camera_index] << " num views = " << num_views
          << "\n view list = " << view_list;

      // Undistort the feature location according the the radial distortion
      // params.
      camera->at(camera_index).pose_.UndistortImagePoint(
          camera->at(camera_index)
              .feature_position_2D_distorted_[sift_key_index],
          &(camera->at(camera_index).feature_position_2D_[sift_key_index]));

      // Set the 3D id to the corresponding world point.
      camera->at(camera_index).feature_3D_ids_[sift_key_index] = i;
    }

    if ((i + 1) % 100 == 0 || i == num_points - 1) {
      std::cout << "\r Loading 3D points " << i + 1 << " / " << num_points
                << std::flush;
    }
  }

  ifs.close();

  return true;
}
