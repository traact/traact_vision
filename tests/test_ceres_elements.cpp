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
#include <traact/math/ceres/PointReprojectionError.h>

TEST(TraactVisionTestSuite, Ceres_Elements_Test) {

    using namespace traact::math;
    using namespace traact::vision;

    CameraCalibration calibration;
    calibration.width = 640;
    calibration.height = 576;
    calibration.fx = 345.62296;
    calibration.fy = 359.71362;
    calibration.skew = 0;
    calibration.cx = 326.55453;
    calibration.cy = 349.54202;




    // in center in front of camera, must be in image center
    {
//        traact::spatial::Pose6D pose_c2w;
//        pose_c2w = traact::vision::Position3D (0, 0, 3);
//        Position3D point_w2p(0, 0, 0);
//
//        auto result_reference = reproject_point(pose_c2w, calibration, point_w2p);
//        PointReprojectionError cost_function(result_reference, pose_c2w, calibration, 0);
//
//        traact::Scalar parameter[3];
//        traact::Scalar residuals[2];
//        parameter[0] = point_w2p.x;
//        parameter[1] = point_w2p.y;
//        parameter[2] = point_w2p.z;
//        cost_function(parameter, residuals);
//
//        EXPECT_EQ(residuals[0], 0);
//        EXPECT_EQ(residuals[1], 0);

    }

    {
        traact::spatial::Pose6D pose_c2w;
        pose_c2w = traact::spatial::Translation3D(-2.13208468683971208e+00, -1.20638214730114912e+00, 2.30451254078307244e+00);
        pose_c2w.rotate(traact::spatial::Rotation3D(-8.49694826269651537e-01,
                                    -3.46199513669736558e-01,
                                    1.29443829434892799e-01,
                                    3.76043739432868451e-01));

//        Position3D point_w2p(0.5, -1, 1);
//        auto result_reference = reproject_point(pose_c2w, calibration, point_w2p);
//        PointReprojectionError cost_function(result_reference, pose_c2w, calibration, 0);
//
//        traact::Scalar parameter[3];
//        traact::Scalar residuals[2];
//        parameter[0] = point_w2p.x;
//        parameter[1] = point_w2p.y;
//        parameter[2] = point_w2p.z;
//        cost_function(parameter, residuals);
//
//        EXPECT_NEAR(residuals[0], 0, 1e-6);
//        EXPECT_NEAR(residuals[1], 0, 1e-6);

    }

}