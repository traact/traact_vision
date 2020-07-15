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

#include "UndistortionHelper.h"
#include <traact/opencv/OpenCVUtils.h>
void
traact::vision::UndistortionHelper::Init(const traact::vision::CameraCalibration &calibration, bool optimize_intrinsics,
                                         bool center_principle_point, double alpha) {

    if(distorted_calibration_ == calibration)
        return;
    distorted_calibration_ = calibration;
    undistorted_calibration_ = calibration;
    undistorted_calibration_.radial_distortion.clear();
    undistorted_calibration_.tangential_distortion.clear();
    cv::Size cv_image_size(calibration.width, calibration.height);
    if(optimize_intrinsics) {
        cv::Mat cv_intrinsics;
        cv::Mat cv_distortion;
        traact2cv(calibration,cv_intrinsics, cv_distortion);

        cv::Mat optimized_intrinsics = cv::getOptimalNewCameraMatrix(cv_intrinsics, cv_distortion, cv_image_size, alpha,cv_image_size, 0, center_principle_point );
        cv2traact(undistorted_calibration_, optimized_intrinsics, cv::Mat(), cv_image_size);
    }

    cv::Mat cv_intrinsics_dis;
    cv::Mat cv_distortion_dis;
    traact2cv(calibration,cv_intrinsics_dis, cv_distortion_dis);

    cv::Mat cv_intrinsics_undis;
    cv::Mat cv_distortion_undis;
    traact2cv(undistorted_calibration_,cv_intrinsics_undis, cv_distortion_undis);

    cv::initUndistortRectifyMap(cv_intrinsics_dis, cv_distortion_dis, cv::Mat(), cv_intrinsics_undis, cv_image_size, CV_16SC2, mapX_, mapY_ );

}

traact::vision::CameraCalibration traact::vision::UndistortionHelper::GetUndistortedCalibration() {
    return undistorted_calibration_;
}

bool traact::vision::UndistortionHelper::UndistortImage(const Image& input, Image& output  ) {
    if(input.IsGpu()) {

    }
    cv::remap( input.GetCpuMat(), output.GetCpuMat(), mapX_, mapY_, cv::INTER_LINEAR );

    return true;
}
