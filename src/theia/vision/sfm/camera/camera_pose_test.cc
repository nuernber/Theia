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
//         Torsten Sattler (sattlert@inf.ethz.ch)

#include <Eigen/Dense>

#include <math.h>
#include "gtest/gtest.h"

#include "theia/alignment/alignment.h"
#include "theia/test/test_utils.h"
#include "theia/vision/sfm/camera/camera_pose.h"

namespace theia {

namespace {

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Matrix;

class CameraPoseTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW protected : virtual void SetUp() {
    // Initializes R with a random rotation matrix.
    //    R_ << -0.5633,  0.7560,  0.3334,
    //         -0.6822, -0.1978, -0.7039,
    //         -0.4662, -0.6239, 0.6272;
    rotation_matrix_.block<1, 3>(0, 0) =
        Vector3d(-0.5633, 0.7560, 0.3334).normalized().transpose();
    rotation_matrix_.block<1, 3>(1, 0) =
        Vector3d(-0.6822, -0.1978, -0.7039).normalized().transpose();
    rotation_matrix_.block<1, 3>(2, 0) =
        Vector3d(-0.4662, -0.6239, 0.6272).normalized().transpose();

    translation_ = Vector3d(0.0, 1.0, -20.0);

    position_ = -rotation_matrix_.transpose() * translation_;

    f_ = -1.0;
    calibration_matrix_ << f_, 0.0, 0.0, 0.0, f_, 0.0, 0.0, 0.0, 1.0;

    k1_ = -2.0;
    k2_ = 3.0;
    k3_ = 5.0;
    k4_ = -3.5;

    transformation_matrix_.linear() = rotation_matrix_;
    transformation_matrix_.translation() = translation_;

    projection_matrix_ = calibration_matrix_ * transformation_matrix_.matrix();

    pose_.InitializePose(rotation_matrix_, translation_, calibration_matrix_,
                         k1_, k2_, k3_, k4_);
  }

  // Checks that a pose was correctly initialized with the parameters given
  // above.
  void TestPose(const CameraPose &pose) {
    const Matrix3d calibration_matrix = pose.calibration_matrix();

    // calibration_matrix_ has the form diag(f,f,1), while calibration_matrix
    // might have the form diag(1,1,1/f).
    // In order to be comparable, we thus introduce a temporary internal
    // camera calibration that we compare against.
    const Matrix3d calibration_matrix_tmp =
        calibration_matrix_ * calibration_matrix(2, 2);
    test::ExpectMatricesNear(calibration_matrix, calibration_matrix_tmp, 1e-6);

    double k1 = 0.0;
    double k2 = 1.0;
    double k3 = 2.0;
    double k4 = 3.0;
    pose.radial_distortion(&k1, &k2, &k3, &k4);
    ASSERT_DOUBLE_EQ(k1_, k1);
    ASSERT_DOUBLE_EQ(k2_, k2);
    ASSERT_DOUBLE_EQ(k3_, k3);
    ASSERT_DOUBLE_EQ(k4_, k4);

    Matrix<double, 3, 4> projection_matrix = pose.projection_matrix();
    // Depending on the form of the internal calibration which was used to
    // initialize the pose, the projection matrix might differ by a scaling
    // factor from projection_matrix_. Again, we use a temporary projection
    // matrix to allow a meaningful comparison.
    Matrix<double, 3, 4> projection_matrix_tmp =
        calibration_matrix_tmp * transformation_matrix_.matrix();
    test::ExpectMatricesNear(projection_matrix, projection_matrix_tmp, 1e-6);

    Matrix<double, 3, 4> T = pose.transformation_matrix();
    test::ExpectMatricesNear(T, transformation_matrix_.matrix(), 1e-6);
  }

  // The rotation matrix of the pose.
  Matrix3d rotation_matrix_;
  // The translation of the pose.
  Vector3d translation_;

  // The position of the pose.
  Vector3d position_;

  // The focal length of the pose.
  double f_;
  // Internal camera calibration matrix.
  Matrix3d calibration_matrix_;
  // The radial distortion parameters of the pose.
  double k1_;
  double k2_;
  double k3_;
  double k4_;
  // The projection matrix of the pose.
  Matrix<double, 3, 4> projection_matrix_;
  // The transformation matrix of the pose.
  Eigen::AffineCompact3d transformation_matrix_;
  // A camera pose for testing.
  CameraPose pose_;
};

TEST_F(CameraPoseTest, InializationFromParameters) {
  CameraPose pose;
  pose.InitializePose(rotation_matrix_, translation_, calibration_matrix_, k1_,
                      k2_, k3_, k4_);
  TestPose(pose);
}

TEST_F(CameraPoseTest, InializationFromOtherPose) {
  CameraPose pose;
  pose.InitializePose(pose_);
  TestPose(pose);
}

TEST_F(CameraPoseTest, InializationFromProjectionMatrix) {
  Matrix3d K;
  K << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;

  Matrix<double, 3, 4> projection_matrix = K * transformation_matrix_.matrix();
  CameraPose pose;
  pose.InitializePose(projection_matrix, k1_, k2_, k3_, k4_);
  TestPose(pose);
}

TEST_F(CameraPoseTest, WorldToCameraWorks) {
  Vector3d p_w = -rotation_matrix_.inverse() * translation_;
  Vector3d p_c(Vector3d::Zero());
  pose_.WorldToCamera(p_w, &p_c);
  ASSERT_NEAR(0.0, p_c[0], 1e-6);
  ASSERT_NEAR(0.0, p_c[1], 1e-6);
  ASSERT_NEAR(0.0, p_c[2], 1e-6);
}

TEST_F(CameraPoseTest, WorldToCameraManyPointsWorks) {
  std::vector<Vector3d> world_points = { Vector3d(1.0, 1.0, 5.0),
                                         Vector3d(-2.3, 1.7, 3.8),
                                         Vector3d(1.9, -0.9, 4.1),
                                         Vector3d(-1.4, -1.9, 3.0) };
  std::vector<Vector3d> gt_camera_points;
  for (int i = 0; i < world_points.size(); i++) {
    gt_camera_points.push_back(transformation_matrix_ * world_points[i]);
  }

  std::vector<Vector3d> camera_points;
  pose_.WorldToCamera(world_points, &camera_points);
  ASSERT_EQ(camera_points.size(), world_points.size());
  for (int i = 0; i < world_points.size(); i++) {
    ASSERT_DOUBLE_EQ(camera_points[i].x(), gt_camera_points[i].x());
    ASSERT_DOUBLE_EQ(camera_points[i].y(), gt_camera_points[i].y());
    ASSERT_DOUBLE_EQ(camera_points[i].z(), gt_camera_points[i].z());
  }
}

TEST_F(CameraPoseTest, CameraToWorldWorks) {
  Vector3d p_w = transformation_matrix_.inverse().translation();
  Vector3d p_c(Vector3d::Zero());
  Vector3d p_w_comp(Vector3d::Zero());
  pose_.CameraToWorld(p_c, &p_w_comp);
  ASSERT_NEAR(p_w[0], p_w_comp[0], 1e-6);
  ASSERT_NEAR(p_w[1], p_w_comp[1], 1e-6);
  ASSERT_NEAR(p_w[2], p_w_comp[2], 1e-6);
}


TEST_F(CameraPoseTest, CameraToWorldManyPointsWorks) {
  std::vector<Vector3d> camera_points = { Vector3d(1.0, 1.0, 5.0),
                                          Vector3d(-2.3, 1.7, 3.8),
                                          Vector3d(1.9, -0.9, 4.1),
                                          Vector3d(-1.4, -1.9, 3.0) };
  std::vector<Vector3d> gt_world_points;
  for (int i = 0; i < camera_points.size(); i++) {
    gt_world_points.push_back(transformation_matrix_.inverse() *
                              camera_points[i]);
  }

  std::vector<Vector3d> world_points;
  pose_.CameraToWorld(camera_points, &world_points);
  ASSERT_EQ(world_points.size(), camera_points.size());
  for (int i = 0; i < world_points.size(); i++) {
    ASSERT_DOUBLE_EQ(world_points[i].x(), gt_world_points[i].x());
    ASSERT_DOUBLE_EQ(world_points[i].y(), gt_world_points[i].y());
    ASSERT_DOUBLE_EQ(world_points[i].z(), gt_world_points[i].z());
  }
}

TEST_F(CameraPoseTest, CameraToImageWorks) {
  Matrix3d R(Matrix3d::Identity());
  Vector3d t(Vector3d::Zero());
  CameraPose pose;
  pose.InitializePose(R, t, calibration_matrix_, 0.0, 0.0, 0.0, 0.0);

  Vector2d p_i;
  pose.CameraToImage(Vector3d(10.0, -20.0, -1.0), &p_i);
  ASSERT_DOUBLE_EQ(10.0, p_i[0]);
  ASSERT_DOUBLE_EQ(-20.0, p_i[1]);

  // Tests that the decision whether the point is in front or behind the
  // camera also works in the case that the intrinsic matrix is given in the
  // form diag(1,1,1/f).
  CameraPose pose2;
  Matrix3d K(Matrix3d::Identity());
  K(2, 2) = -0.5;
  pose2.InitializePose(R, t, K, 0.0, 0.0, 0.0, 0.0);
  pose2.CameraToImage(Vector3d(10.0, -20.0, -1.0), &p_i);
  ASSERT_DOUBLE_EQ(p_i.x(), 20.0);
  ASSERT_DOUBLE_EQ(p_i.y(), -40.0);

  K(2, 2) = 0.5;
  pose2.InitializePose(R, t, K, 0.0, 0.0, 0.0, 0.0);
  pose2.CameraToImage(Vector3d(10.0, 10.0, 1.0), &p_i);
  ASSERT_DOUBLE_EQ(p_i.x(), 20.0);
  ASSERT_DOUBLE_EQ(p_i.y(), 20.0);
}

TEST_F(CameraPoseTest, CameraToImageManyPointsWorks) {
  Matrix3d R(Matrix3d::Identity());
  Vector3d t(Vector3d::Zero());
  CameraPose pose;
  pose.InitializePose(R, t, calibration_matrix_, 0.0, 0.0, 0.0, 0.0);

  std::vector<Vector3d> p_world = {
    Vector3d(1.0, 1.0, 2.0), Vector3d(-5.0, 0.2, 1.0), Vector3d(1.6, -0.9, 1.2)
  };

  std::vector<Vector2d> gt_p_image = { Vector2d(-1.0 / 2.0, -1.0 / 2.0),
                                       Vector2d(5.0 / 1.0, -0.2 / 1.0),
                                       Vector2d(-1.6 / 1.2, 0.9 / 1.2) };

  std::vector<Vector2d> p_image;
  pose.CameraToImage(p_world, &p_image);
  ASSERT_EQ(p_image.size(), p_world.size());
  for (int i = 0; i < p_image.size(); i++) {
    ASSERT_DOUBLE_EQ(p_image[i].x(), gt_p_image[i].x());
    ASSERT_DOUBLE_EQ(p_image[i].y(), gt_p_image[i].y());
  }
}

TEST_F(CameraPoseTest, WorldToImageWorks) {
  Matrix3d R(Matrix3d::Identity());
  Vector3d t(Vector3d::Zero());
  CameraPose pose;
  pose.InitializePose(R, t, calibration_matrix_, 0.0, 0.0, 0.0, 0.0);

  Vector2d p_i(0.0, 0.0);
  ASSERT_FALSE(pose.WorldToImage(Vector3d(10.0, 10.0, 0.0), &p_i));
  ASSERT_FALSE(pose.WorldToImage(Vector3d(10.0, 10.0, 1.0), &p_i));
  ASSERT_TRUE(pose.WorldToImage(Vector3d(10.0, -20.0, -1.0), &p_i));
  ASSERT_NEAR(10.0, p_i[0], 1e-6);
  ASSERT_NEAR(-20.0, p_i[1], 1e-6);

  // Tests that the decision whether the point is in front or behind the
  // camera also works in the case that the intrinsic matrix is given in the
  // form diag(1,1,1/f).
  CameraPose pose2;
  Matrix3d K(Matrix3d::Identity());
  K(2, 2) = -0.5;
  pose2.InitializePose(R, t, K, 0.0, 0.0, 0.0, 0.0);
  ASSERT_FALSE(pose2.WorldToImage(Vector3d(10.0, 10.0, 0.0), &p_i));
  ASSERT_FALSE(pose2.WorldToImage(Vector3d(10.0, 10.0, 1.0), &p_i));
  ASSERT_TRUE(pose2.WorldToImage(Vector3d(10.0, -20.0, -1.0), &p_i));

  K(2, 2) = 0.5;
  pose2.InitializePose(R, t, K, 0.0, 0.0, 0.0, 0.0);
  ASSERT_FALSE(pose2.WorldToImage(Vector3d(10.0, 10.0, 0.0), &p_i));
  ASSERT_TRUE(pose2.WorldToImage(Vector3d(10.0, 10.0, 1.0), &p_i));
  ASSERT_FALSE(pose2.WorldToImage(Vector3d(10.0, -20.0, -1.0), &p_i));
}

TEST_F(CameraPoseTest, WorldToImageManyPointsWorks) {
  Matrix3d R(Matrix3d::Identity());
  Vector3d t(Vector3d::Zero());
  CameraPose pose;
  pose.InitializePose(R, t, calibration_matrix_, 0.0, 0.0, 0.0, 0.0);

  std::vector<Vector3d> p_world = {
    Vector3d(1.0, 1.0, 2.0), Vector3d(-5.0, 0.2, 1.0), Vector3d(1.6, -0.9, 1.2)
  };

  std::vector<Vector2d> gt_p_image = { Vector2d(-1.0 / 2.0, -1.0 / 2.0),
                                       Vector2d(5.0 / 1.0, -0.2 / 1.0),
                                       Vector2d(-1.6 / 1.2, 0.9 / 1.2) };

  std::vector<Vector2d> p_image;
  pose.WorldToImage(p_world, &p_image);
  ASSERT_EQ(p_image.size(), p_world.size());
  for (int i = 0; i < p_image.size(); i++) {
    ASSERT_DOUBLE_EQ(p_image[i].x(), gt_p_image[i].x());
    ASSERT_DOUBLE_EQ(p_image[i].y(), gt_p_image[i].y());
  }
}

TEST_F(CameraPoseTest, UndistortImagePoint) {
  Matrix3d R(Matrix3d::Identity());
  Vector3d t(Vector3d::Zero());
  CameraPose pose;
  pose.InitializePose(R, t, calibration_matrix_, 1.0, 0.0, 0.0, 0.0);

  Vector2d p_u(1.0, 1.0);
  pose.UndistortImagePoint(Vector2d(0.0, 0.0), &p_u);
  ASSERT_NEAR(0.0, p_u[0], 1e-6);
  ASSERT_NEAR(0.0, p_u[1], 1e-6);

  pose.UndistortImagePoint(Vector2d(1.0, 2.0), &p_u);
  ASSERT_NEAR(1.0 / 6.0, p_u[0], 1e-6);
  ASSERT_NEAR(2.0 / 6.0, p_u[1], 1e-6);

  pose.InitializePose(R, t, calibration_matrix_, -10.0, 0.0, 0.0, 0.0);
  pose.UndistortImagePoint(Vector2d(0.0, 0.0), &p_u);
  ASSERT_NEAR(0.0, p_u[0], 1e-6);
  ASSERT_NEAR(0.0, p_u[1], 1e-6);

  pose.UndistortImagePoint(Vector2d(5.0, 3.0), &p_u);
  ASSERT_NEAR(5.0 / (-339.0), p_u[0], 1e-6);
  ASSERT_NEAR(3.0 / (-339.0), p_u[1], 1e-6);
}

TEST_F(CameraPoseTest, UndistortImagePointManyPoints) {
  Matrix3d R(Matrix3d::Identity());
  Vector3d t(Vector3d::Zero());
  CameraPose pose;
  pose.InitializePose(R, t, calibration_matrix_, 1.0, 0.0, 0.0, 0.0);

  std::vector<Vector2d> distorted_points = { Vector2d(0.0, 0.0),
                                             Vector2d(1.0, 2.0),
                                             Vector2d(-5.0, 3.0) };
  std::vector<Vector2d> gt_undistorted_points = {
    Vector2d(0.0, 0.0), Vector2d(1.0 / 6.0, 2.0 / 6.0),
    Vector2d(-5.0 / 35.0, 3.0 / 35.0)
  };

  std::vector<Vector2d> undistorted_points;
  pose.UndistortImagePoint(distorted_points, &undistorted_points);
  ASSERT_EQ(undistorted_points.size(), distorted_points.size());
  for (int i = 0; i < distorted_points.size(); i++) {
    ASSERT_DOUBLE_EQ(undistorted_points[i].x(), gt_undistorted_points[i].x());
    ASSERT_DOUBLE_EQ(undistorted_points[i].y(), gt_undistorted_points[i].y());
  }


  pose.InitializePose(R, t, calibration_matrix_, -2.0, 0.0, 0.0, 0.0);
  gt_undistorted_points = { Vector2d(0.0, 0.0),
                            Vector2d(1.0 / -9.0, 2.0 / -9.0),
                            Vector2d(-5.0 / -67.0, 3.0 / -67.0) };
  pose.UndistortImagePoint(distorted_points, &undistorted_points);
  ASSERT_EQ(undistorted_points.size(), distorted_points.size());
  for (int i = 0; i < distorted_points.size(); i++) {
    ASSERT_DOUBLE_EQ(undistorted_points[i].x(), gt_undistorted_points[i].x());
    ASSERT_DOUBLE_EQ(undistorted_points[i].y(), gt_undistorted_points[i].y());
  }
}

}  // namespace

}  // namespace theia
