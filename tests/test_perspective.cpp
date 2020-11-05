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

#include "gtest/gtest.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <traact/math/perspective.h>
#include <traact/vision.h>
#include <traact/spatial.h>

#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

TEST(TraactVisionTestSuite, ReprojectionTest_NoDistortion) {

    using namespace traact::math;
    using namespace traact::vision;
    using namespace Eigen;
    CameraCalibration calibration;
    calibration.width = 640;
    calibration.height = 480;
    calibration.fx = 500;
    calibration.fy = 500;
    calibration.skew = 0;
    calibration.cx = calibration.width / 2.0 - 0.5;
    calibration.cy = calibration.height / 2.0 - 0.5;

    cv::Mat opencv_intrinsics(3, 3, cv::DataType<double>::type);
    opencv_intrinsics.at<double>(0,0) = calibration.fx;
    opencv_intrinsics.at<double>(1,0) = 0;
    opencv_intrinsics.at<double>(2,0) = 0;

    opencv_intrinsics.at<double>(0,1) = 0;
    opencv_intrinsics.at<double>(1,1) = calibration.fy;
    opencv_intrinsics.at<double>(2,1) = 0;

    opencv_intrinsics.at<double>(0,2) = calibration.cx;
    opencv_intrinsics.at<double>(1,2) = calibration.cy;
    opencv_intrinsics.at<double>(2,2) = 1;

    cv::Mat opencv_distortion(4,1,cv::DataType<double>::type);
    opencv_distortion.at<double>(0) = 0;
    opencv_distortion.at<double>(1) = 0;
    opencv_distortion.at<double>(2) = 0;
    opencv_distortion.at<double>(3) = 0;

    cv::Mat tVec(3, 1, cv::DataType<double>::type);
    tVec.at<double>(0) = 0;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = 0;

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    rvec.at<double>(0) = 0;
    rvec.at<double>(1) = 0;
    rvec.at<double>(1) = 0;


    // in center in front of camera, must be in image center
    {
        std::vector<cv::Point3d> opencv_point;
        std::vector<cv::Point2d> opencv_result;
        opencv_point.push_back(cv::Point3d(0,0,-1));
        Translation3d point(0,0,-1);

        cv::projectPoints(opencv_point, rvec, tVec, opencv_intrinsics, opencv_distortion, opencv_result);

        auto result = reproject_point(calibration, point.translation());
        EXPECT_EQ(result.x(), 319.5);
        EXPECT_EQ(result.y(), 239.5);

        EXPECT_EQ(result.x(), opencv_result[0].x);
        EXPECT_EQ(result.y(), opencv_result[0].y);
    }


    // test four points
    {
        std::vector<cv::Point3d> opencv_point;
        std::vector<cv::Point2d> opencv_result;
        opencv_point.emplace_back(cv::Point3d(1,1,1));
        opencv_point.emplace_back(-1.5,1.2,2);
        opencv_point.emplace_back(cv::Point3d(-1,1.5,3));
        opencv_point.emplace_back(cv::Point3d(1,-2,4));

        cv::projectPoints(opencv_point, rvec, tVec, opencv_intrinsics, opencv_distortion, opencv_result);

        for(int i=0;i<opencv_point.size();++i) {
            const auto opencv_p = opencv_point[i];
            Translation3d point(opencv_p.x,opencv_p.y,opencv_p.z);

            auto result = reproject_point(calibration, point.translation());
            EXPECT_DOUBLE_EQ(result.x(), opencv_result[i].x);
            EXPECT_DOUBLE_EQ(result.y(), opencv_result[i].y);
        }

    }




}