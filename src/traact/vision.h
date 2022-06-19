/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_H_

#include "vision_datatypes.h"
#include "traact/traact.h"
#include "traact/vision/Image.h"
#include <traact/spatial.h>

namespace traact::vision {

using FeatureID = uint64_t;

TRAACT_VISION_EXPORT FeatureID createFeatureId();

using Position2D = cv::Point_<traact::Scalar>;
using Position3D = cv::Point3_<traact::Scalar>;
using Position2DList = std::vector<Position2D>;
using Position3DList = std::vector<Position3D>;
using KeyPointList = std::vector<cv::KeyPoint>;

struct TRAACT_VISION_EXPORT Feature {
    cv::Mat descriptor;
    FeatureID feature_id;
    std::vector<FeatureID> constructed_from;

    void createIds();
};

struct TRAACT_VISION_EXPORT FeatureList {
    cv::Mat descriptor;
    std::vector<FeatureID> feature_id;
    std::unordered_map<FeatureID, std::vector<FeatureID>> constructed_from;

    void createIds(size_t count);
    void createIds(size_t count, FeatureID created_from);
    void clear();
};

CREATE_TRAACT_HEADER_TYPE(Position2DHeader, traact::vision::Position2D, "vision:Position2D", TRAACT_VISION_EXPORT)
CREATE_TRAACT_HEADER_TYPE(Position3DHeader, traact::vision::Position3D, "vision:Position3D", TRAACT_VISION_EXPORT)
CREATE_TRAACT_HEADER_TYPE(Position2DListHeader, traact::vision::Position2DList, "vision:Position2DList", TRAACT_VISION_EXPORT)
CREATE_TRAACT_HEADER_TYPE(Position3DListHeader, traact::vision::Position3DList, "vision:Position3DList", TRAACT_VISION_EXPORT)
CREATE_TRAACT_HEADER_TYPE(KeyPointListHeader, traact::vision::KeyPointList, "vision:KeyPointList", TRAACT_VISION_EXPORT)
CREATE_TRAACT_HEADER_TYPE(FeatureHeader, traact::vision::Feature, "vision:Feature", TRAACT_VISION_EXPORT)
CREATE_TRAACT_HEADER_TYPE(FeatureListHeader, traact::vision::FeatureList, "vision:FeatureList", TRAACT_VISION_EXPORT)

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


int TRAACT_VISION_EXPORT getOpenCvDepth(BaseType type);

}


#define CREATE_VISION_COMPONENTS(external_component) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, ImageHeader) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, CameraCalibrationHeader) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, Position2DHeader) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, Position3DHeader) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, Position2DListHeader) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::vision, Position3DListHeader)


#define REGISTER_VISION_COMPONENTS(external_component) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, ImageHeader) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, CameraCalibrationHeader) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, Position2DHeader) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, Position3DHeader) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, Position2DListHeader) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, Position3DListHeader)


#endif //TRAACT_VISION_SRC_TRAACT_VISION_H_
