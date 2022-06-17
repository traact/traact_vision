/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "gtest/gtest.h"
#include <traact/util/Logging.h>
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

    cv::Mat opencv_intrinsics(3, 3, cv::DataType<traact::Scalar>::type);
    opencv_intrinsics.at<traact::Scalar>(0, 0) = calibration.fx;
    opencv_intrinsics.at<traact::Scalar>(1, 0) = 0;
    opencv_intrinsics.at<traact::Scalar>(2, 0) = 0;

    opencv_intrinsics.at<traact::Scalar>(0, 1) = 0;
    opencv_intrinsics.at<traact::Scalar>(1, 1) = calibration.fy;
    opencv_intrinsics.at<traact::Scalar>(2, 1) = 0;

    opencv_intrinsics.at<traact::Scalar>(0, 2) = calibration.cx;
    opencv_intrinsics.at<traact::Scalar>(1, 2) = calibration.cy;
    opencv_intrinsics.at<traact::Scalar>(2, 2) = 1;

    cv::Mat opencv_distortion(4, 1, cv::DataType<traact::Scalar>::type);
    opencv_distortion.at<traact::Scalar>(0) = 0;
    opencv_distortion.at<traact::Scalar>(1) = 0;
    opencv_distortion.at<traact::Scalar>(2) = 0;
    opencv_distortion.at<traact::Scalar>(3) = 0;

    cv::Mat tVec(3, 1, cv::DataType<traact::Scalar>::type);
    tVec.at<traact::Scalar>(0) = 0;
    tVec.at<traact::Scalar>(1) = 0;
    tVec.at<traact::Scalar>(2) = 0;

    cv::Mat rvec(3, 1, cv::DataType<traact::Scalar>::type);
    rvec.at<traact::Scalar>(0) = 0;
    rvec.at<traact::Scalar>(1) = 0;
    rvec.at<traact::Scalar>(1) = 0;


    // in center in front of camera, must be in image center
    {
        std::vector<cv::Point3d> opencv_point;
        std::vector<cv::Point2d> opencv_result;
        opencv_point.push_back(cv::Point3d(0, 0, -1));
        Eigen::Vector3<traact::Scalar> point(0, 0, -1);

        cv::projectPoints(opencv_point, rvec, tVec, opencv_intrinsics, opencv_distortion, opencv_result);

        auto result = reproject_point(calibration, point);
        EXPECT_EQ(result.x(), 319.5);
        EXPECT_EQ(result.y(), 239.5);

        EXPECT_EQ(result.x(), opencv_result[0].x);
        EXPECT_EQ(result.y(), opencv_result[0].y);
    }


    // test four points
    {
        std::vector<cv::Point3d> opencv_point;
        std::vector<cv::Point2d> opencv_result;
        opencv_point.emplace_back(cv::Point3d(1, 1, 1));
        opencv_point.emplace_back(-1.5, 1.2, 2);
        opencv_point.emplace_back(cv::Point3d(-1, 1.5, 3));
        opencv_point.emplace_back(cv::Point3d(1, -2, 4));

        cv::projectPoints(opencv_point, rvec, tVec, opencv_intrinsics, opencv_distortion, opencv_result);

        for (int i = 0; i < opencv_point.size(); ++i) {
            const auto opencv_p = opencv_point[i];
            Eigen::Vector3<traact::Scalar> point(opencv_p.x, opencv_p.y, opencv_p.z);

            auto result = reproject_point(calibration, point);
            EXPECT_DOUBLE_EQ(result.x(), opencv_result[i].x);
            EXPECT_DOUBLE_EQ(result.y(), opencv_result[i].y);
        }

    }

}