/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_UTILS_H
#define TRAACTMULTI_UTILS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <traact/vision_datatypes.h>

namespace traact {
static inline void traact2eigen(const vision::CameraCalibration &calibration, Eigen::Matrix3d &intrinsics) {
    intrinsics.setIdentity();
    intrinsics(0, 0) = calibration.fx;
    intrinsics(1, 1) = calibration.fy;
    intrinsics(0, 2) = calibration.cx;
    intrinsics(1, 2) = calibration.cy;
    intrinsics(0, 1) = calibration.skew;
}

}

#endif //TRAACTMULTI_UTILS_H

