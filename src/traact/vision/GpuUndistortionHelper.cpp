/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "traact/vision/GpuUndistortionHelper.h"
#include "traact/opencv/OpenCVUtils.h"
#include <traact/util/Logging.h>

#include <opencv2/cudawarping.hpp>

namespace traact::vision {
void
GpuUndistortionHelper::init(const CameraCalibration &calibration,
                            bool optimize_intrinsics,
                            bool center_principle_point,
                            traact::Scalar alpha) {

    // init is not thread safe while the undistortion is
    std::lock_guard init_guard(init_mutex_);
    if (distorted_calibration_ == calibration)
        return;
    distorted_calibration_ = calibration;
    undistorted_calibration_ = calibration;
    undistorted_calibration_.radial_distortion.clear();
    undistorted_calibration_.tangential_distortion.clear();
    cv::Size cv_image_size(calibration.width, calibration.height);
    if (optimize_intrinsics) {
        SPDLOG_INFO("optimize intrinsics");
        cv::Mat cv_intrinsics;
        cv::Mat cv_distortion;
        traact2cv(calibration, cv_intrinsics, cv_distortion);

        cv::Mat optimized_intrinsics = cv::getOptimalNewCameraMatrix(cv_intrinsics,
                                                                     cv_distortion,
                                                                     cv_image_size,
                                                                     alpha,
                                                                     cv_image_size,
                                                                     0,
                                                                     center_principle_point);
        cv2traact(undistorted_calibration_, optimized_intrinsics, cv::Mat(), cv_image_size);
    } else {
        SPDLOG_INFO("keep intrinsics");
    }

    cv::Mat cv_intrinsics_dis;
    cv::Mat cv_distortion_dis;
    traact2cv(calibration, cv_intrinsics_dis, cv_distortion_dis);

    cv::Mat cv_intrinsics_undis;
    cv::Mat cv_distortion_undis;
    traact2cv(undistorted_calibration_, cv_intrinsics_undis, cv_distortion_undis);

    std::stringstream intrinsic_message;
    intrinsic_message << "from intrinscis:\n";
    intrinsic_message << cv_intrinsics_dis;
    intrinsic_message << "\n";

    intrinsic_message << "to intrinscis:\n";
    intrinsic_message << cv_intrinsics_undis;
    intrinsic_message << "\n";
    SPDLOG_INFO(intrinsic_message.str());

    cv::initUndistortRectifyMap(cv_intrinsics_dis,
                                cv_distortion_dis,
                                cv::Mat::eye(3, 3, CV_32F),
                                cv_intrinsics_undis,
                                cv_image_size,
                                CV_32FC1,
                                cpu_map_x_,
                                cpu_map_y_);

    map_x_.create(cv_image_size, CV_32FC1 );
    map_y_.create(cv_image_size, CV_32FC1 );

    uploaded_ = false;


}

CameraCalibration GpuUndistortionHelper::getUndistortedCalibration() {
    return undistorted_calibration_;
}


GpuUndistortionHelper::GpuUndistortionHelper(const GpuUndistortionHelper &other) {
    undistorted_calibration_ = other.undistorted_calibration_;
    distorted_calibration_ = other.distorted_calibration_;
    map_x_ = other.map_x_.clone();
    map_y_ = other.map_y_.clone();

}

bool GpuUndistortionHelper::undistortImage(const cv::cuda::GpuMat &input,
                                           cv::cuda::GpuMat &output,
                                           cv::cuda::Stream &stream) {

    cv::cuda::remap(input, output, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream);

    return true;
}
void GpuUndistortionHelper::reset() {
    distorted_calibration_ = CameraCalibration();

}
void GpuUndistortionHelper::init(cv::cuda::Stream &stream) {
    if(!uploaded_){
        map_x_.upload(cpu_map_x_, stream);
        map_y_.upload(cpu_map_y_, stream);
        uploaded_ = true;
    }

}

}
