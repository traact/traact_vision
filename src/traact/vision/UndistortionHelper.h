/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_UNDISTORTIONHELPER_H
#define TRAACTMULTI_UNDISTORTIONHELPER_H

#include "traact/vision_datatypes.h"
#include <mutex>

namespace traact::vision {

class UndistortionHelper {
 public:
    UndistortionHelper() = default;
    UndistortionHelper(const UndistortionHelper &other);
    ~UndistortionHelper() = default;

    void Init(const CameraCalibration &calibration,
              bool optimize_intrinsics = false,
              bool center_principle_point = true,
              traact::Scalar alpha = 0);

    CameraCalibration GetUndistortedCalibration();

    bool UndistortImage(const cv::Mat &input, cv::Mat &output);

 protected:
    cv::Mat mapX_;
    cv::Mat mapY_;
    CameraCalibration distorted_calibration_;
    CameraCalibration undistorted_calibration_;
    std::mutex init_mutex_;

};

}

#endif //TRAACTMULTI_UNDISTORTIONHELPER_H
