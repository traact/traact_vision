/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "vision.h"


namespace traact::vision {

int getOpenCvDepth(BaseType type) {
    switch (type) {

        case INT_8:return CV_8S;
        case UINT_8:return CV_8U;
        case INT_16:return CV_16S;
        case UINT_16:return CV_16U;
        case INT_32:return CV_32S;
        case FLOAT_16: return CV_16F;
        case FLOAT_32:return CV_32F;
        case FLOAT_64:return CV_64F;
        case UNKNOWN:
        default: {
            SPDLOG_ERROR("unknown BaseType {0}", static_cast<int>(type));
            return CV_8U;
        }
    }
}

int getOpenCvType(const ImageHeader &header) {
    return CV_MAKETYPE(getOpenCvDepth(header.base_type), header.channels);
}
void setOpenCvType(const cv::Mat &opencv_type, ImageHeader &header) {
    header.width = opencv_type.cols;
    header.height = opencv_type.rows;
    header.stride = opencv_type.step;

    auto opencv_depth = opencv_type.type() & CV_MAT_DEPTH_MASK;
    auto channels = 1 + (opencv_type.type() >> CV_CN_SHIFT);
    header.channels = channels;

    switch (opencv_depth) {

        case CV_8U: header.base_type = BaseType::UINT_8;
            break;
        case CV_8S: header.base_type = BaseType::INT_8;
            break;
        case CV_16U: header.base_type = BaseType::UINT_16;
            break;
        case CV_16S: header.base_type = BaseType::INT_16;
            break;
        case CV_32S: header.base_type = BaseType::INT_32;
            break;
        case CV_16F: header.base_type = BaseType::FLOAT_16;
            break;
        case CV_32F: header.base_type = BaseType::FLOAT_32;
            break;
        case CV_64F: header.base_type = BaseType::FLOAT_64;
            break;
        default: {
            SPDLOG_ERROR("unknown opencv depth type {0}", opencv_depth);
            header.base_type = BaseType::UNKNOWN;
            break;
        }
    }
}

void ImageHeader::copyFrom(const ImageHeader &header) {
    width = header.width;
    height = header.height;
    channels = header.channels;
    stride = header.stride;
    pixel_format = header.pixel_format;
    base_type = header.base_type;
}
}

namespace traact::component::facade {
CREATE_VISION_COMPONENTS(ApplicationAsyncSource)
CREATE_VISION_COMPONENTS(ApplicationSyncSink)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::CameraCalibrationHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::ImageHeader)
    REGISTER_VISION_COMPONENTS(traact::component::facade::ApplicationAsyncSource)
    REGISTER_VISION_COMPONENTS(traact::component::facade::ApplicationSyncSink)
END_TRAACT_PLUGIN_REGISTRATION