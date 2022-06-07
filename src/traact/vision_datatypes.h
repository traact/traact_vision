/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

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

class TRAACT_VISION_EXPORT Image;

struct TRAACT_VISION_EXPORT ImageHeader {
    ImageHeader() {}
    /**
       * Definitions needed by traact and the user to use a datatype
       */
    static const char *MetaType;
    typedef Image NativeType;
    static const char *NativeTypeName;

    PixelFormat pixel_format = PixelFormat::UNKNOWN_PIXELFORMAT;
    int width = 0;
    int height = 0;
    int opencv_matrix_type = 0;
    size_t opencv_step = cv::Mat::AUTO_STEP;
    int device_id = 0;
};

struct TRAACT_VISION_EXPORT CameraCalibration {
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;
    double skew;
    std::vector<double> radial_distortion;
    std::vector<double> tangential_distortion;

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

struct TRAACT_VISION_EXPORT CameraCalibrationHeader {
    CameraCalibrationHeader() {}
    /**
       * Definitions needed by traact and the user to use a datatype
       */
    static const char *MetaType;
    typedef CameraCalibration NativeType;
    static const char *NativeTypeName;

};

class TRAACT_VISION_EXPORT Image {
 public:
    Image();
    Image(ImageHeader header, bool is_cpu);
    ~Image() = default;

    bool init(ImageHeader header);

    cv::Mat &GetCpuMat();
    cv::cuda::GpuMat &GetGpuMat();

    const cv::Mat &GetCpuMat() const;
    void SetCpuMat(cv::Mat &image);
    const cv::cuda::GpuMat &GetGpuMat() const;

    cv::Mat GetCpuCopy() const;

    ImageHeader GetHeader() const;
    bool IsCpu() const;
    bool IsGpu() const;
 private:
    ImageHeader header_;
    bool is_cpu_;
    cv::Mat cpu_mat_;
    bool is_gpu_;
    cv::cuda::GpuMat gpu_mat_;

};

class TRAACT_VISION_EXPORT ImageFactoryObject : public buffer::TemplatedDefaultFactoryObject<ImageHeader> {

};

class TRAACT_VISION_EXPORT CameraCalibrationFactoryObject : public buffer::TemplatedDefaultFactoryObject<
    CameraCalibrationHeader> {

};

}

#endif //TRAACTMULTI_VISION_DATATYPES_H
