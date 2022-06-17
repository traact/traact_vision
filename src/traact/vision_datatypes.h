/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_VISION_DATATYPES_H
#define TRAACTMULTI_VISION_DATATYPES_H

#include <traact/buffer/DataFactory.h>
#include <traact/datatypes.h>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <traact/traact_vision_export.h>
#include <ostream>

namespace traact::vision {

enum class TRAACT_VISION_EXPORT PixelFormat {
    UNKNOWN_PIXELFORMAT = 0,
    LUMINANCE,
    RGB,
    BGR,
    RGBA,
    BGRA,
    YUV422,
    YUV411,
    RAW,
    DEPTH,
    FLOAT,
    MJPEG
};

struct TRAACT_VISION_EXPORT CameraCalibration {
    int width;
    int height;
    traact::Scalar fx;
    traact::Scalar fy;
    traact::Scalar cx;
    traact::Scalar cy;
    traact::Scalar skew;
    std::vector<traact::Scalar> radial_distortion;
    std::vector<traact::Scalar> tangential_distortion;

    bool operator==(const CameraCalibration &rhs) const {
        return width == rhs.width &&
            height == rhs.height &&
            fx == rhs.fx &&
            fy == rhs.fy &&
            cx == rhs.cx &&
            cy == rhs.cy &&
            skew == rhs.skew &&
            radial_distortion == rhs.radial_distortion &&
            tangential_distortion == rhs.tangential_distortion;
    }

    bool operator!=(const CameraCalibration &rhs) const {
        return !(rhs == *this);
    }

    friend std::ostream &operator<<(std::ostream &os, const CameraCalibration &calibration) {
        os << "width: " << calibration.width << " height: " << calibration.height << std::endl
           << " fx: " << calibration.fx << " fy: " << calibration.fy << std::endl
           << " cx: " << calibration.cx << " cy: " << calibration.cy << std::endl
           << " skew: " << calibration.skew << std::endl;

        if (calibration.radial_distortion.empty()) {
            os << " radial distortion: ";
            for (auto r : calibration.radial_distortion) {
                os << r << " ";
            }
            os << std::endl;
        } else {
            os << "no radial distortion " << std::endl;
        }

        if (calibration.tangential_distortion.empty()) {
            os << "no tangential distortion " << std::endl;

        } else {
            os << " tangential distortion: ";//
            for (auto r : calibration.tangential_distortion) {
                os << r << " ";
            }
        }

        return os;
    }

};






}

#endif //TRAACTMULTI_VISION_DATATYPES_H
