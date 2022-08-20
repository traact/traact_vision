/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_GPUUNDISTORTIONHELPER_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_GPUUNDISTORTIONHELPER_H_

#include "traact/vision_datatypes.h"
#include <mutex>
#include <opencv2/opencv.hpp>

namespace traact::vision {

class UndistortionHelper {
 public:
    UndistortionHelper() = default;
    UndistortionHelper(const UndistortionHelper &other);
    ~UndistortionHelper() = default;

    void init(const CameraCalibration &calibration,
              bool optimize_intrinsics = false,
              bool center_principle_point = true,
              traact::Scalar alpha = 0);

    CameraCalibration getUndistortedCalibration();

    bool undistortImage(const cv::Mat &input, cv::Mat &output);
    void reset();

 protected:
    cv::Mat mapX_;
    cv::Mat mapY_;
    CameraCalibration distorted_calibration_;
    CameraCalibration undistorted_calibration_;
    std::mutex init_mutex_;

};

}

#endif //TRAACT_VISION_SRC_TRAACT_VISION_GPUUNDISTORTIONHELPER_H_
