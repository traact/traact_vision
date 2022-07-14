/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_GPUIMAGE_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_GPUIMAGE_H_

#include "traact/traact_vision_export.h"
#include <traact/util/ValueWrapper.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
namespace traact::vision {


class TRAACT_VISION_EXPORT GPUImage : public util::ValueWrapper<cv::cuda::GpuMat>{
 public:
    GPUImage() = default;
    GPUImage(GPUImage const& image) = default;
    GPUImage& operator=(GPUImage const& image) =default;
    GPUImage(GPUImage && image) = default;
    GPUImage& operator=(GPUImage && image) = default;
};

} // traact



#endif //TRAACT_VISION_SRC_TRAACT_VISION_GPUIMAGE_H_
