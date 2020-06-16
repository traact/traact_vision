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

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_VISION_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_VISION_H_

#include <traact/buffer/GenericFactoryObject.h>
#include <traact/buffer/GenericBufferTypeConversion.h>
#include <traact/datatypes.h>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

namespace traact::vision {

enum class PixelFormat {
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

class Image;

struct ImageHeader {
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

struct CameraCalibrationHeader {
  CameraCalibrationHeader() {}
  /**
     * Definitions needed by traact and the user to use a datatype
     */
  static const char *MetaType;
  typedef Image NativeType;
  static const char *NativeTypeName;

};

class Image {
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

  cv::Mat &&GetCpuCopy() const;


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

class ImageFactoryObject : public buffer::GenericFactoryObject {
 public:
  std::string getTypeName() override {
    return std::move(std::string(ImageHeader::MetaType));
  }
  void *createObject(void *) override {
    return new ImageHeader::NativeType;
  }
  void deleteObject(void *obj) override {
    auto *tmp = static_cast<ImageHeader::NativeType *>(obj);
    delete tmp;
  }

};

}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_VISION_H_
