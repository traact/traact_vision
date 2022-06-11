/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_VISION_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_VISION_H_

#include "vision_datatypes.h"
#include "traact/traact.h"
#include "traact/vision/Image.h"

namespace traact {

void *getData(cv::Mat &buffer);

size_t getDataLength(cv::Mat &buffer);


void *getData(std::vector<uint8_t> &buffer);

size_t getDataLength(std::vector<uint8_t> &buffer);

}

namespace traact::vision {
CREATE_TRAACT_HEADER_TYPE(CameraCalibrationHeader,
                          traact::vision::CameraCalibration,
                          "vision:CameraCalibration",
                          TRAACT_VISION_EXPORT)

struct TRAACT_VISION_EXPORT ImageHeader {
    ImageHeader() = default;
    using NativeType = Image;
    static constexpr const char *NativeTypeName{"traact::vision::Image"};
    static constexpr const char *MetaType{"vision:Image"};
    const size_t size = sizeof(Image);
    int width{0};
    int height{0};
    int channels{0};
    size_t stride{0};
    PixelFormat pixel_format{PixelFormat::UNKNOWN_PIXELFORMAT};
    BaseType base_type{BaseType::UNKNOWN};

    void initData(Image& image, const ImageHeader& request);
    void copyFrom(const ImageHeader &header);
};

class TRAACT_VISION_EXPORT ImageHeaderFactory : public traact::buffer::TemplatedDefaultDataFactory<ImageHeader> {
 TRAACT_PLUGIN_ENABLE(traact::buffer::TemplatedDefaultDataFactory<ImageHeader>, traact::buffer::DataFactory)
};

int getOpenCvType(const ImageHeader &header);
void setOpenCvType(const cv::Mat &opencv_type, ImageHeader &header);

int getOpenCvDepth(BaseType type);

}

template<>
[[maybe_unused]] inline cv::Mat getBufferAs(const traact::vision::ImageHeader &header, const traact::vision::Image &data){
    return data.getImage();
}

template<>
[[maybe_unused]] inline cv::Mat getBufferAs(const traact::vision::ImageHeader &header, traact::vision::Image &data){
    return data.getImage();
}


#define CREATE_VISION_COMPONENTS(external_component) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, ImageHeader) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, CameraCalibrationHeader)


#define REGISTER_VISION_COMPONENTS(external_component) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, ImageHeader) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, CameraCalibrationHeader)


#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_VISION_H_
