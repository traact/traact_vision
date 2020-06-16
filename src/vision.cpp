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

#include <traact/vision.h>

namespace traact::vision {

const char * ImageHeader::NativeTypeName =  "traact::vision::Image";
const char * ImageHeader::MetaType =  "vision:Image";

const char * CameraCalibrationHeader::NativeTypeName =  "traact::vision::CameraIntrinsic";
const char * CameraCalibrationHeader::MetaType =  "vision:CameraIntrinsics";
}

traact::vision::Image::Image() : is_cpu_(false), is_gpu_(false) {

}
traact::vision::Image::Image(ImageHeader header, bool is_cpu) : header_(std::move(header)), is_cpu_(is_cpu), is_gpu_(!is_cpu) {
  init(header);
}

cv::Mat &traact::vision::Image::GetCpuMat() {
  if(!is_cpu_ && is_gpu_){
    cv::cuda::setDevice(header_.device_id);
    gpu_mat_.download(cpu_mat_);
    is_cpu_ = true;
  }

  return cpu_mat_;
}
cv::cuda::GpuMat &traact::vision::Image::GetGpuMat() {
  if(!is_gpu_ && is_cpu_){
    cv::cuda::setDevice(header_.device_id);
    gpu_mat_.upload(cpu_mat_);
    is_gpu_ = true;
  }
  return gpu_mat_;
}

traact::vision::ImageHeader traact::vision::Image::GetHeader() const {
  return header_;
}

bool traact::vision::Image::init(traact::vision::ImageHeader header) {


  if(is_cpu_)
    cpu_mat_ = cv::Mat(header.height, header.width, header.opencv_matrix_type, header.opencv_step);
  if(is_gpu_){
    cv::cuda::setDevice(header.device_id);
    gpu_mat_ = cv::cuda::GpuMat(header.height, header.width, header.opencv_matrix_type, header.opencv_step);
  }

  header_ = std::move(header);
  return true;
}
bool traact::vision::Image::IsCpu() const {
  return is_cpu_;
}
bool traact::vision::Image::IsGpu() const {
  return is_gpu_;
}
const cv::Mat &traact::vision::Image::GetCpuMat() const {
  return cpu_mat_;
}
const cv::cuda::GpuMat &traact::vision::Image::GetGpuMat() const {
  return gpu_mat_;
}
cv::Mat &&traact::vision::Image::GetCpuCopy() const {
  if(is_cpu_) {
    cv::Mat result = cpu_mat_.clone();
    return std::move(result);
  }

  if(is_gpu_) {
    cv::Mat result;
    cv::cuda::setDevice(header_.device_id);
    gpu_mat_.download(result);
    return std::move(result);
  }
  assert("image is not initialized");
}
void traact::vision::Image::SetCpuMat(cv::Mat &image) {
  cpu_mat_ = image;
  is_cpu_ = true;

}

namespace traact::buffer {
template<>
vision::ImageHeader::NativeType &GenericBufferTypeConversion::asMutable<vision::ImageHeader::NativeType,
                                                                        vision::ImageHeader>(void *obj,
                                                                                                 void *header) {
  return *static_cast<vision::ImageHeader::NativeType *>(obj);
}

template<>
const vision::ImageHeader::NativeType &GenericBufferTypeConversion::asImmutable<vision::ImageHeader::NativeType,
                                                                                vision::ImageHeader>(void *obj,
                                                                                                         void *header) {
  return *static_cast<vision::ImageHeader::NativeType *>(obj);
}

}
