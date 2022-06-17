/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "UndistortionHelper.h"
#include "traact/opencv/OpenCVUtils.h"
#include <traact/util/Logging.h>
namespace traact::vision {
void
UndistortionHelper::Init(const CameraCalibration &calibration, bool optimize_intrinsics,
                         bool center_principle_point, traact::Scalar alpha) {

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

    std::cout << "from intrinscis:\n";
    std::cout << cv_intrinsics_dis;
    std::cout << "\n";

    std::cout << "to intrinscis:\n";
    std::cout << cv_intrinsics_undis;
    std::cout << "\n";

    cv::initUndistortRectifyMap(cv_intrinsics_dis,
                                cv_distortion_dis,
                                cv::Mat::eye(3, 3, CV_32F),
                                cv_intrinsics_undis,
                                cv_image_size,
                                CV_16SC2,
                                mapX_,
                                mapY_);

}

CameraCalibration UndistortionHelper::GetUndistortedCalibration() {
    return undistorted_calibration_;
}


UndistortionHelper::UndistortionHelper(const UndistortionHelper &other) {
    undistorted_calibration_ = other.undistorted_calibration_;
    distorted_calibration_ = other.distorted_calibration_;
    mapX_ = other.mapX_.clone();
    mapY_ = other.mapY_.clone();

}

bool UndistortionHelper::UndistortImage(const cv::Mat &input, cv::Mat &output) {
    cv::remap(input, output, mapX_, mapY_, cv::INTER_LINEAR);

    return true;
}

}
