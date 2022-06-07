/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#include <traact/vision.h>

namespace traact::vision {

const char *ImageHeader::NativeTypeName = "traact::vision::Image";
const char *ImageHeader::MetaType = "vision:Image";

const char *CameraCalibrationHeader::NativeTypeName = "traact::vision::CameraIntrinsic";
const char *CameraCalibrationHeader::MetaType = "vision:CameraCalibration";
}

traact::vision::Image::Image() : is_cpu_(false), is_gpu_(false) {

}
traact::vision::Image::Image(ImageHeader header, bool is_cpu)
    : header_(std::move(header)), is_cpu_(is_cpu), is_gpu_(!is_cpu) {
    init(header);
}

cv::Mat &traact::vision::Image::GetCpuMat() {
    if (!is_cpu_ && is_gpu_) {
        cv::cuda::setDevice(header_.device_id);
        gpu_mat_.download(cpu_mat_);
        is_cpu_ = true;
    }

    return cpu_mat_;
}
cv::cuda::GpuMat &traact::vision::Image::GetGpuMat() {
    if (!is_gpu_ && is_cpu_) {
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


    //if(is_cpu_)
    is_cpu_ = true;
    cpu_mat_ = cv::Mat(header.height, header.width, header.opencv_matrix_type, header.opencv_step);
    //if(is_gpu_){
    //  cv::cuda::setDevice(header.device_id);
    // gpu_mat_ = cv::cuda::GpuMat(header.height, header.width, header.opencv_matrix_type, header.opencv_step);
    //}

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
cv::Mat traact::vision::Image::GetCpuCopy() const {
    if (is_cpu_) {
        return cpu_mat_.clone();
    }

    if (is_gpu_) {
        cv::Mat result;
        cv::cuda::setDevice(header_.device_id);
        gpu_mat_.download(result);
        return result;
    }
    assert("image is not initialized");
    return cv::Mat();
}
void traact::vision::Image::SetCpuMat(cv::Mat &image) {
    cpu_mat_ = image;
    is_cpu_ = true;

}
