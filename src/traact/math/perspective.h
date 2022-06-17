/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_PERSPECTIVE_H
#define TRAACTMULTI_PERSPECTIVE_H

#include "traact/vision_datatypes.h"
#include <traact/datatypes.h>

#include "utils.h"
#include <traact/spatial.h>

namespace traact::math {


Eigen::Vector2<traact::Scalar> reproject_point(const vision::CameraCalibration &intrinsics, const Eigen::Vector3<traact::Scalar> &point);
Eigen::Vector2<traact::Scalar> reproject_point(const traact::spatial::Pose6D &cam2world,
                                const traact::vision::CameraCalibration &intrinsics,
                                const Eigen::Vector3<traact::Scalar> &point);

bool estimate_camera_pose(spatial::Pose6D &pose_result,
                          const spatial::Position2DList &image_points,
                          const vision::CameraCalibration &intrinsics,
                          const spatial::Position3DList &model_points);

traact::Scalar reprojection_error(const traact::spatial::Pose6D &cam2world,
                          const spatial::Position2D &image_points,
                          const traact::vision::CameraCalibration &intrinsics,
                          const spatial::Position3D &model_points);
traact::Scalar average_reprojection_error(const traact::spatial::Pose6D &cam2world,
                                  const spatial::Position2DList &image_points,
                                  const traact::vision::CameraCalibration &intrinsics,
                                  const spatial::Position3DList &model_points);

bool estimate_3d_point(Eigen::Vector3<traact::Scalar> &result,
                       const std::vector<traact::spatial::Pose6D> &cam2world,
                       const std::vector<vision::CameraCalibration> &intrinsics,
                       const std::vector<Eigen::Vector2<traact::Scalar>> &image_point,
                       double *covariance_output = 0);

bool estimate_3d_pose(spatial::Pose6D &result,
                      const std::vector<traact::spatial::Pose6D> &cam2world,
                      const std::vector<vision::CameraCalibration> &intrinsics,
                      const std::vector<spatial::Position2DList> &image_point,
                      const spatial::Position3DList &model,
                      double *covariance_output = 0);

bool estimate_3d_pose(traact::spatial::Pose6D &result, const traact::spatial::Position3DList &src,
                      const traact::spatial::Position3DList &dst, double *covariance_output = 0);

Eigen::Matrix<traact::Scalar, 3, 4> create_projection_matrix(const traact::spatial::Pose6D &cam2world,
                                                     const vision::CameraCalibration &calibration);

void undistort_points(const vision::CameraCalibration &dis_calibration,
                      const vision::CameraCalibration &undis_calibration,
                      const std::vector<Eigen::Vector2<traact::Scalar>> &distorted_points,
                      std::vector<Eigen::Vector2<traact::Scalar>> &undistorted_points);
}

#endif //TRAACTMULTI_PERSPECTIVE_H
