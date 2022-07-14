/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_IMAGE_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_IMAGE_H_

#include "traact/traact_vision_export.h"
#include <traact/util/ValueWrapper.h>
#include <opencv2/opencv.hpp>

namespace traact::vision {

 class TRAACT_VISION_EXPORT Image : public util::ValueWrapper<cv::Mat>{
    public:
    Image() = default;
    Image(Image const& image) = default;
    Image& operator=(Image const& image) =default;
    Image(Image && image) = default;
    Image& operator=(Image && image) = default;
};

} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_IMAGE_H_
