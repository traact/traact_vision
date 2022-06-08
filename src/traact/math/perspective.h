/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_PERSPECTIVE_H
#define TRAACTMULTI_PERSPECTIVE_H

#include "traact/vision_datatypes.h"
#include <traact/datatypes.h>

#include "utils.h"
#include <traact/spatial.h>

namespace traact::math {
Eigen::Vector2d reproject_point(const vision::CameraCalibration &intrinsics, const Eigen::Vector3d &point);
Eigen::Vector2d reproject_point(const Eigen::Affine3d &cam2world,
                                const traact::vision::CameraCalibration &intrinsics,
                                const Eigen::Vector3d &point);

bool estimate_camera_pose(spatial::Pose6D &pose_result,
                          const spatial::Position2DList &image_points,
                          const vision::CameraCalibration &intrinsics,
                          const spatial::Position3DList &model_points);

double reprojection_error(const Eigen::Affine3d &cam2world,
                          const spatial::Position2D &image_points,
                          const traact::vision::CameraCalibration &intrinsics,
                          const spatial::Position3D &model_points);
double average_reprojection_error(const Eigen::Affine3d &cam2world,
                                  const spatial::Position2DList &image_points,
                                  const traact::vision::CameraCalibration &intrinsics,
                                  const spatial::Position3DList &model_points);

bool estimate_3d_point(Eigen::Vector3d &result,
                       const std::vector<Eigen::Affine3d> &cam2world,
                       const std::vector<vision::CameraCalibration> &intrinsics,
                       const std::vector<Eigen::Vector2d> &image_point,
                       double *covariance_output = 0);

bool estimate_3d_pose(spatial::Pose6D &result,
                      const std::vector<Eigen::Affine3d> &cam2world,
                      const std::vector<vision::CameraCalibration> &intrinsics,
                      const std::vector<spatial::Position2DList> &image_point,
                      const spatial::Position3DList &model,
                      double *covariance_output = 0);

bool estimate_3d_pose(traact::spatial::Pose6D &result, const traact::spatial::Position3DList &src,
                      const traact::spatial::Position3DList &dst, double *covariance_output = 0);

Eigen::Matrix<double, 3, 4> create_projection_matrix(const Eigen::Affine3d &cam2world,
                                                     const vision::CameraCalibration &calibration);

void undistort_points(const vision::CameraCalibration &dis_calibration,
                      const vision::CameraCalibration &undis_calibration,
                      const std::vector<Eigen::Vector2d> &distorted_points,
                      std::vector<Eigen::Vector2d> &undistorted_points);
}

#endif //TRAACTMULTI_PERSPECTIVE_H
