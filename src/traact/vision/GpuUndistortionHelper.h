/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_GPUGpuUndistortionHelper_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_GPUGpuUndistortionHelper_H_

#include "traact/vision_datatypes.h"
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

namespace traact::vision {

class GpuUndistortionHelper {
 public:
    GpuUndistortionHelper() = default;
    GpuUndistortionHelper(const GpuUndistortionHelper &other);
    ~GpuUndistortionHelper() = default;

    void init(const CameraCalibration &calibration,
              bool optimize_intrinsics = false,
              bool center_principle_point = true,
              traact::Scalar alpha = 0);

    CameraCalibration getUndistortedCalibration();

    bool undistortImage(const cv::cuda::GpuMat &input, cv::cuda::GpuMat &output, cv::cuda::Stream &stream);
    void reset();

 protected:
    cv::cuda::GpuMat mapX_;
    cv::cuda::GpuMat mapY_;
    CameraCalibration distorted_calibration_;
    CameraCalibration undistorted_calibration_;
    std::mutex init_mutex_;

};

}

#endif //TRAACT_VISION_SRC_TRAACT_VISION_GPUGpuUndistortionHelper_H_
