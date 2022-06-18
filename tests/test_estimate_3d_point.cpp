/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#include "gtest/gtest.h"
#include <traact/util/Logging.h>
#include "spdlog/sinks/stdout_color_sinks.h"

#include <traact/math/perspective.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <Eigen/Dense>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

Eigen::Vector3<traact::Scalar> test_point(const std::vector<traact::spatial::Pose6D> &cam_2_world,
                           const std::vector<traact::vision::CameraCalibration> &calibrations,
                           const Eigen::Vector3<traact::Scalar> &test_position,
                           const traact::Scalar noise = 0) {
    using namespace traact::math;
    using namespace traact::vision;
    using namespace Eigen;

    std::vector<Vector2<traact::Scalar>> image_points;
    for (int i = 0; i < cam_2_world.size(); ++i) {
//        Vector2<traact::Scalar> point = reproject_point(cam_2_world[i], calibrations[i], test_position);
//        Vector2<traact::Scalar> pixel_noise;
//        pixel_noise.setRandom();
//        pixel_noise *= noise;
//        point += pixel_noise;
//
//        image_points.push_back(point);
    }

    Vector3<traact::Scalar> p3_result;

    //estimate_3d_point(p3_result, cam_2_world, calibrations, image_points);

    return p3_result;

}

TEST(TraactVisionTestSuite, Estimate3dPointTest_NoDistortion) {

    using namespace traact::math;
    using namespace traact::vision;
    using namespace Eigen;
    CameraCalibration calibration;
    calibration.width = 640;
    calibration.height = 576;
    calibration.fx = 345.62296;
    calibration.fy = 359.71362;
    calibration.skew = 0;
    calibration.cx = 326.55453;
    calibration.cy = 349.54202;

    {
        traact::spatial::Pose6D marker_pose;
        std::vector<traact::spatial::Pose6D> cam_2_world;
        marker_pose = traact::spatial::Translation3D(0.1, -0.5, -3);// * AngleAxis<traact::Scalar>(M_PI, Vector3<traact::Scalar>::UnitY());

        cam_2_world.push_back(traact::spatial::Pose6D::Identity());
        cam_2_world.push_back(traact::spatial::Translation3D(-0.1, 0, 0) * AngleAxis<traact::Scalar> ::Identity());

        std::vector<CameraCalibration> calibrations;
        for (int i = 0; i < cam_2_world.size(); ++i) {
            calibrations.push_back(calibration);
        }

        Vector3<traact::Scalar> p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), 0);
        EXPECT_NEAR(marker_pose.translation().x(), p3_result.x(), 1e-9);
        EXPECT_NEAR(marker_pose.translation().y(), p3_result.y(), 1e-9);
        EXPECT_NEAR(marker_pose.translation().z(), p3_result.z(), 1e-9);
    }

    {
        traact::spatial::Pose6D marker_pose;
        std::vector<traact::spatial::Pose6D> cam_2_world;
        marker_pose = traact::spatial::Translation3D(0.1, -0.5, -3);// * AngleAxis<traact::Scalar>(M_PI, Vector3<traact::Scalar>::UnitY());

        {
            traact::spatial::Pose6D pose_c2w;
            pose_c2w = traact::spatial::Translation3D(-2.13208468683971208e+00, -1.20638214730114912e+00, 2.30451254078307244e+00);
            pose_c2w.rotate(traact::spatial::Rotation3D(-8.49694826269651537e-01,
                                        -3.46199513669736558e-01,
                                        1.29443829434892799e-01,
                                        3.76043739432868451e-01));
            cam_2_world.push_back(pose_c2w);
        }

        {
            traact::spatial::Pose6D pose_c2w;
            pose_c2w = traact::spatial::Translation3D(-2.24341394331049226e+00, 2.91498703341871979e+00, 2.19838501838564992e+00);
            pose_c2w.rotate(traact::spatial::Rotation3D(4.53584848555412923e-01,
                                        1.66481146382847056e-01,
                                        -3.40200132181601056e-01,
                                        -8.06727142919858475e-01));
            cam_2_world.push_back(pose_c2w);
        }

        std::vector<CameraCalibration> calibrations;
        for (int i = 0; i < cam_2_world.size(); ++i) {
            calibrations.push_back(calibration);
        }

        Vector3<traact::Scalar> p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), 0);
        EXPECT_NEAR(marker_pose.translation().x(), p3_result.x(), 1e-9);
        EXPECT_NEAR(marker_pose.translation().y(), p3_result.y(), 1e-9);
        EXPECT_NEAR(marker_pose.translation().z(), p3_result.z(), 1e-9);
    }

}

/*
TEST(TraactVisionTestSuite, Estimate3dPointTestNoise_NoDistortion) {
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

    traact::spatial::Pose6D marker_pose;
    std::vector<traact::spatial::Pose6D>  world_2_cam;
    std::vector<traact::spatial::Pose6D>  cam_2_world;
    marker_pose = traact::spatial::Translation3D(0.1,-0.5,-3);// * AngleAxis<traact::Scalar>(M_PI, Vector3<traact::Scalar>::UnitY());

    world_2_cam.push_back(traact::spatial::Pose6D::Identity());
    world_2_cam.push_back(traact::spatial::Translation3D(-0.1,0,0) * AngleAxis<traact::Scalar>::Identity());

    std::vector<CameraCalibration> calibrations;
    for(int i=0;i<world_2_cam.size();++i){
        calibrations.push_back(calibration);
        cam_2_world.push_back(world_2_cam[i].inverse());
    }

    typedef Matrix< traact::Scalar, Dynamic, 1, ColMajor > EVector;
    Matrix< traact::Scalar, 100, 1, ColMajor > diff_values;

    for(int i=0;i<11;++i) {
        traact::Scalar noise = 10.0/100.0 * i;

        traact::Scalar sum=0;
        const int test_size = 1000;
        for(int j=0;j<test_size;++j) {
            Vector3<traact::Scalar> p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), noise);
            Vector3<traact::Scalar> diff = p3_result - marker_pose.translation();
            sum += diff.norm()*1000;
        }


        SPDLOG_INFO("2 observations, noise of {0} pixel : error {1}mm", noise, sum/test_size);
    }

    world_2_cam.push_back(traact::spatial::Translation3D(0,0,-6) *  AngleAxis<traact::Scalar>(M_PI, Vector3<traact::Scalar>::UnitY()));
    world_2_cam.push_back(traact::spatial::Translation3D(-0.1,0,-6) *  AngleAxis<traact::Scalar>(M_PI, Vector3<traact::Scalar>::UnitY()));

    calibrations.clear();
    cam_2_world.clear();
    for(int i=0;i<world_2_cam.size();++i){
        calibrations.push_back(calibration);
        cam_2_world.push_back(world_2_cam[i].inverse());
    }

    for(int i=0;i<11;++i) {
        traact::Scalar noise = 10.0/100.0 * i;

        traact::Scalar sum=0;
        const int test_size = 1000;
        for(int j=0;j<test_size;++j) {
            Vector3<traact::Scalar> p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), noise);
            Vector3<traact::Scalar> diff = p3_result - marker_pose.translation();
            sum += diff.norm()*1000;
        }


        SPDLOG_INFO("4 observations, noise of {0} pixel : error {1}mm", noise, sum/test_size);
    }


}
 */