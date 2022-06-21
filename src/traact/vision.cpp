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
FeatureID createFeatureId() {
    static std::atomic<FeatureID> generator{0};
    return generator++;
}


int getOpenCvType(const ImageHeader &header) {
    return CV_MAKETYPE(getOpenCvDepth(header.base_type), header.channels);
}

void ImageHeader::copyFrom(const ImageHeader &header) {
    width = header.width;
    height = header.height;
    channels = header.channels;
    stride = header.stride;
    pixel_format = header.pixel_format;
    base_type = header.base_type;
}
void ImageHeader::setFrom(const cv::Mat &opencv_type) {
    width = opencv_type.cols;
    height = opencv_type.rows;
    stride = opencv_type.step;

    auto opencv_depth = opencv_type.type() & CV_MAT_DEPTH_MASK;
    channels = 1 + (opencv_type.type() >> CV_CN_SHIFT);

    switch (opencv_depth) {
        case CV_8U: base_type = BaseType::UINT_8;
            break;
        case CV_8S: base_type = BaseType::INT_8;
            break;
        case CV_16U: base_type = BaseType::UINT_16;
            break;
        case CV_16S: base_type = BaseType::INT_16;
            break;
        case CV_32S: base_type = BaseType::INT_32;
            break;
        case CV_16F: base_type = BaseType::FLOAT_16;
            break;
        case CV_32F: base_type = BaseType::FLOAT_32;
            break;
        case CV_64F: base_type = BaseType::FLOAT_64;
            break;
        default: {
            SPDLOG_ERROR("unknown opencv depth type {0}", opencv_depth);
            base_type = BaseType::UNKNOWN;
            break;
        }
    }
}
void Feature::createIds() {
    feature_id = createFeatureId();
    constructed_from.clear();
    descriptor = cv::Mat();
}
void FeatureList::createIds(size_t count) {
    clear();
    feature_id.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        feature_id.template emplace_back(createFeatureId());
    }
}
void FeatureList::createIds(size_t count, FeatureID created_from) {
    createIds(count);
    for (size_t i = 0; i < count; ++i) {
        constructed_from.template emplace(feature_id[i], std::vector<FeatureID>{created_from});
    }
}
void FeatureList::clear() {
    feature_id.clear();
    constructed_from.clear();
    descriptor = cv::Mat();
}
}

namespace traact::component::facade {
CREATE_VISION_COMPONENTS(ApplicationAsyncSource)
CREATE_VISION_COMPONENTS(ApplicationSyncSink)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::CameraCalibrationHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::ImageHeader)

    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::Position2DHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::Position3DHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::Position2DListHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::Position3DListHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::FeatureHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::FeatureListHeader)
    REGISTER_DEFAULT_TRAACT_TYPE(traact::vision::KeyPointListHeader)

    REGISTER_VISION_COMPONENTS(traact::component::facade::ApplicationAsyncSource)
    REGISTER_VISION_COMPONENTS(traact::component::facade::ApplicationSyncSink)
END_TRAACT_PLUGIN_REGISTRATION