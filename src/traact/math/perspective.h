/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_PERSPECTIVE_H
#define TRAACTMULTI_PERSPECTIVE_H

#include "traact/vision_datatypes.h"
#include <traact/datatypes.h>

#include "utils.h"
#include "traact/vision.h"
#include <traact/spatial.h>

namespace traact::math {


traact::vision::Position2D reproject_point(const vision::CameraCalibration &intrinsics, const vision::Position3D &point);
traact::vision::Position2D reproject_point(const traact::spatial::Pose6D &cam2world,
                                           const traact::vision::CameraCalibration &intrinsics,
                                           const vision::Position3D &point);

bool estimate_camera_pose(spatial::Pose6D &pose_result,
                          const vision::Position2DList &image_points,
                          const vision::CameraCalibration &intrinsics,
                          const vision::Position3DList &model_points);

bool estimate_3d_point(vision::Position3D  &result,
                       const std::vector<traact::spatial::Pose6D> &cam2world,
                       const std::vector<vision::CameraCalibration> &intrinsics,
                       const vision::Position2DList &image_point,
                       double *covariance_output = 0);

bool estimate_3d_pose(spatial::Pose6D &result,
                      const std::vector<traact::spatial::Pose6D> &cam2world,
                      const std::vector<vision::CameraCalibration> &intrinsics,
                      const std::vector<vision::Position2DList> &image_point,
                      const vision::Position3DList &model,
                      double *covariance_output = 0);

Eigen::Matrix<traact::Scalar, 3, 4> create_projection_matrix(const traact::spatial::Pose6D &cam2world,
                                                     const vision::CameraCalibration &calibration);

void undistort_points(const vision::CameraCalibration &dis_calibration,
                      const vision::CameraCalibration &undis_calibration,
                      const std::vector<Eigen::Vector2<traact::Scalar>> &distorted_points,
                      std::vector<Eigen::Vector2<traact::Scalar>> &undistorted_points);

traact::Scalar reprojectionError(const vision::Position2DList &image_points,
                                 const vision::CameraCalibration &intrinsics,
                                 const vision::Position3DList &camera_to_points);

traact::Scalar reprojectionError(const traact::spatial::Pose6D &camera_to_world,
                                 const vision::Position2DList &image_points,
                                 const vision::CameraCalibration &intrinsics,
                                 const vision::Position3DList &world_to_points);

traact::Scalar reprojectionError(const traact::spatial::Pose6D &camera_to_world,
                                 const vision::Position2D &image_points,
                                 const vision::CameraCalibration &intrinsics,
                                 const vision::Position3D &world_to_points);
}

#endif //TRAACTMULTI_PERSPECTIVE_H
