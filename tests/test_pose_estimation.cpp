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

void test_pose(const Eigen::Affine3d &marker_pose, const traact::vision::CameraCalibration &calibration) {
    using namespace traact::math;
    using namespace traact::vision;
    using namespace Eigen;

    std::vector<Translation3d> marker_corners_local;
    std::vector<Affine3d> marker_corners;


    /*
    marker_corners_local.push_back(Translation3d(0.2,0.2,0));
    marker_corners_local.push_back(Translation3d(0.2,-0.2,0));
    marker_corners_local.push_back(Translation3d(-0.2,-0.2,0));
    marker_corners_local.push_back(Translation3d(-0.2,0.2,0));
*/


    marker_corners_local.push_back(Eigen::Translation3d(0, 0, 0));
    marker_corners_local.push_back(Eigen::Translation3d(0.384, 0, 0));
    marker_corners_local.push_back(Eigen::Translation3d(0, 0.114, 0));
    marker_corners_local.push_back(Eigen::Translation3d(0.225, 0, 0));

    traact::spatial::Position2DList marker_corners_2d;
    traact::spatial::Position3DList marker_model;
    // project marker corners to 2d
    for (int i = 0; i < marker_corners_local.size(); ++i) {
        marker_corners.push_back(marker_pose * marker_corners_local[i]);
        marker_corners_2d.emplace_back(reproject_point(calibration, marker_corners[i].translation()));
        marker_model.push_back(marker_corners_local[i].translation());
    }

    Affine3d camera_pose, pose_result;
    EXPECT_TRUE(traact::math::estimate_camera_pose(pose_result, marker_corners_2d, calibration, marker_model));
    //pose_result = camera_pose.inverse();

    EXPECT_NEAR(marker_pose.translation().x(), pose_result.translation().x(), 1e-6);
    EXPECT_NEAR(marker_pose.translation().y(), pose_result.translation().y(), 1e-6);
    EXPECT_NEAR(marker_pose.translation().z(), pose_result.translation().z(), 1e-6);

    Quaterniond rot_marker(marker_pose.rotation());
    Quaterniond rot_result(pose_result.rotation());

    EXPECT_NEAR(rot_marker.x(), rot_result.x(), 1e-6);
    EXPECT_NEAR(rot_marker.y(), rot_result.y(), 1e-6);
    EXPECT_NEAR(rot_marker.z(), rot_result.z(), 1e-6);
    EXPECT_NEAR(rot_marker.w(), rot_result.w(), 1e-6);
}

TEST(TraactVisionTestSuite, PoseEstimationTest_NoDistortion) {

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

    Affine3d marker_pose;
    marker_pose = Translation3d(1, 2, 3);
    //test_pose(marker_pose, calibration);
    marker_pose.rotate(AngleAxisd(M_PI, Vector3d::UnitZ()));
    //test_pose(marker_pose, calibration);

    marker_pose = Translation3d(0.1, 0.2, 4) * AngleAxisd(M_PI / 16, Vector3d::UnitY());
    //test_pose(marker_pose, calibration);

    marker_pose = Translation3d(0.3, -0.5, 2) * AngleAxisd(M_PI / 16, Vector3d::UnitX());
    //test_pose(marker_pose, calibration);


    for (int i = 0; i < 100; ++i) {
        Vector3d pos;
        Vector4d rot;
        pos.setRandom();
        rot.setRandom();
        Quaterniond rotq(rot);
        rotq.normalize();
        pos.z() += 3;

        Affine3d pose = Translation3d(pos) * rotq;
        //test_pose(pose, calibration);

    }

}
