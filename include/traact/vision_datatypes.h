/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef TRAACTMULTI_VISION_DATATYPES_H
#define TRAACTMULTI_VISION_DATATYPES_H

#include <traact/buffer/GenericFactoryObject.h>
#include <traact/buffer/GenericBufferTypeConversion.h>
#include <traact/datatypes.h>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <traact/traact_vision_export.h>

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
        void SetCpuMat(cv::Mat& image);
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

    class TRAACT_VISION_EXPORT CameraCalibrationFactoryObject : public buffer::TemplatedDefaultFactoryObject<CameraCalibrationHeader> {

    };

}

#endif //TRAACTMULTI_VISION_DATATYPES_H
