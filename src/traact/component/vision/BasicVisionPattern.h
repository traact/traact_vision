/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_BASICVISIONPATTERN_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_BASICVISIONPATTERN_H_

#include <traact/pattern/Pattern.h>
#include <traact/datatypes.h>
#include <traact/vision.h>
namespace traact::component::vision {
static traact::pattern::Pattern::Ptr getUncalibratedCameraPattern() {

    traact::pattern::Pattern::Ptr
        pattern =
        std::make_shared<traact::pattern::Pattern>("UncalibratedCameraPattern", Concurrency::SERIAL);

    pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);
    pattern->addCoordinateSystem("ImagePlane")
        .addCoordinateSystem("Image", true)
        .addEdge("ImagePlane", "Image", "output");

    std::set<std::string> pixel_formats;
    pixel_formats.emplace("Luminance");
    pattern->addParameter("width", 640, 1)
        .addParameter("height", 320, 1)
        .addParameter("PixelFormat", "Luminance", pixel_formats);

    return pattern;
}

static traact::pattern::Pattern::Ptr getCameraPattern() {

    traact::pattern::Pattern::Ptr
        pattern = getUncalibratedCameraPattern();

    pattern->addProducerPort("output_calibration", traact::vision::CameraCalibrationHeader::MetaType);

    pattern->addCoordinateSystem("Camera")
        .addEdge("ImagePlane", "Camera", "intrinsic");

    return pattern;
}

}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_BASICVISIONPATTERN_H_
