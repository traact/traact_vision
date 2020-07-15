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
#include <Eigen/Dense>
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void test_pose(const Eigen::Affine3d& marker_pose, const traact::vision::CameraCalibration& calibration) {
    using namespace traact::math;
    using namespace traact::vision;
    using namespace Eigen;

    std::vector<Translation3d> marker_corners_local;
    std::vector<Affine3d> marker_corners;

    marker_corners_local.push_back(Translation3d(0.2,0.2,0));
    marker_corners_local.push_back(Translation3d(0.2,-0.2,0));
    marker_corners_local.push_back(Translation3d(-0.2,-0.2,0));
    marker_corners_local.push_back(Translation3d(-0.2,0.2,0));

    std::vector<Vector2d> marker_corners_2d;
    std::vector<Vector3d> marker_model;
    // project marker corners to 2d
    for(int i=0;i<marker_corners_local.size();++i) {
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
    marker_pose = Translation3d(1,2,-3);
    test_pose(marker_pose, calibration);
    marker_pose.rotate(AngleAxisd(M_PI, Vector3d::UnitZ()));
    test_pose(marker_pose, calibration);

    marker_pose = Translation3d(0.1,0.2,-4) * AngleAxisd(M_PI/16, Vector3d::UnitY());
    test_pose(marker_pose, calibration);

    marker_pose = Translation3d(0.3,-0.5,-2) * AngleAxisd(M_PI/16, Vector3d::UnitX());
    test_pose(marker_pose, calibration);


    for(int i=0;i<100;++i) {
        Vector3d pos;
        Vector4d rot;
        pos.setRandom();
        rot.setRandom();
        Quaterniond rotq(rot);
        rotq.normalize();
        pos.z() += -3;

        Affine3d pose = Translation3d (pos) * rotq;
        test_pose(pose, calibration);

    }




}
