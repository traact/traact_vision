/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#ifndef TRAACTMULTI_GENERATEMULTICAMERABATESTDATA_H
#define TRAACTMULTI_GENERATEMULTICAMERABATESTDATA_H

#include <traact/spatial.h>
#include "traact/vision.h"

namespace traact::util {

struct GenerateMultiCameraBATestDataCamera {
    spatial::Pose6D camera2world;
    spatial::Pose6D camera2world_noise;
    vision::CameraCalibration calibration;

};
class GenerateMultiCameraBATestData {
 public:
    void Init(size_t data_count, size_t camera_count, size_t width, size_t height,
              spatial::Position3DList model_points, double point2d_noise, double camera_pos_noise,
              double camera_rot_noise,
              size_t noise_point_count);
    std::vector<GenerateMultiCameraBATestDataCamera> GetCameraData();

    spatial::Position2DList GetPointsForCamera(size_t idx);
    bool Next();

    spatial::Pose6D GetTargetPose(size_t data_idx);
    spatial::Position2DList GetPointsForCamera(size_t data_idx, size_t camera_idx);
    spatial::Position2DList GetPointsForCameraNoise(size_t data_idx, size_t camera_idx);

 protected:
    size_t max_count_;
    size_t current_count_{0};
    double point2d_noise_{0};
    double camera_pos_noise_{0};
    double camera_rot_noise_{0};
    std::vector<GenerateMultiCameraBATestDataCamera> camera_data_;
    std::vector<spatial::Pose6D> expected_pose_;
    std::vector<std::vector<spatial::Position2DList>> reference_point2d_;
    std::vector<std::vector<spatial::Position2DList>> reference_point2d_noise_;
    spatial::Position3DList model_points_;

};
}

#endif //TRAACTMULTI_GENERATEMULTICAMERABATESTDATA_H
