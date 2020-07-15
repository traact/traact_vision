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

Eigen::Vector3d test_point(const std::vector<Eigen::Affine3d>&  cam_2_world, const std::vector<traact::vision::CameraCalibration>& calibrations, const Eigen::Vector3d& test_position, const double noise=0) {
    using namespace traact::math;
    using namespace traact::vision;
    using namespace Eigen;

    std::vector<Vector2d> image_points;
    for(int i=0;i<cam_2_world.size();++i){
        Vector2d point = reproject_point(cam_2_world[i], calibrations[i], test_position);
        Vector2d pixel_noise;
        pixel_noise.setRandom();
        pixel_noise *= noise;
        point += pixel_noise;

        image_points.push_back(point);
    }

    Vector3d p3_result;


    estimate_3d_point(p3_result,cam_2_world, calibrations, image_points);

    return p3_result;

}

TEST(TraactVisionTestSuite, Estimate3dPointTest_NoDistortion) {

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



    {
        Affine3d marker_pose;
        std::vector<Affine3d>  cam_2_world;
        marker_pose = Translation3d(0.1,-0.5,-3);// * AngleAxisd(M_PI, Vector3d::UnitY());

        cam_2_world.push_back(Affine3d::Identity());
        cam_2_world.push_back(Translation3d(-0.1,0,0) * AngleAxisd::Identity());

        std::vector<CameraCalibration> calibrations;
        for(int i=0;i<cam_2_world.size();++i){
            calibrations.push_back(calibration);
        }

        Vector3d p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), 0);
        EXPECT_NEAR(marker_pose.translation().x(), p3_result.x(), 1e-9);
        EXPECT_NEAR(marker_pose.translation().y(), p3_result.y(), 1e-9);
        EXPECT_NEAR(marker_pose.translation().z(), p3_result.z(), 1e-9);
    }




}

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

    Affine3d marker_pose;
    std::vector<Affine3d>  world_2_cam;
    std::vector<Affine3d>  cam_2_world;
    marker_pose = Translation3d(0.1,-0.5,-3);// * AngleAxisd(M_PI, Vector3d::UnitY());

    world_2_cam.push_back(Affine3d::Identity());
    world_2_cam.push_back(Translation3d(-0.1,0,0) * AngleAxisd::Identity());

    std::vector<CameraCalibration> calibrations;
    for(int i=0;i<world_2_cam.size();++i){
        calibrations.push_back(calibration);
        cam_2_world.push_back(world_2_cam[i].inverse());
    }

    typedef Matrix< double, Dynamic, 1, ColMajor > EVector;
    Matrix< double, 100, 1, ColMajor > diff_values;

    for(int i=0;i<11;++i) {
        double noise = 10.0/100.0 * i;

        double sum=0;
        const int test_size = 1000;
        for(int j=0;j<test_size;++j) {
            Vector3d p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), noise);
            Vector3d diff = p3_result - marker_pose.translation();
            sum += diff.norm()*1000;
        }


        SPDLOG_INFO("2 observations, noise of {0} pixel : error {1}mm", noise, sum/test_size);
    }

    world_2_cam.push_back(Translation3d(0,0,-6) *  AngleAxisd(M_PI, Vector3d::UnitY()));
    world_2_cam.push_back(Translation3d(-0.1,0,-6) *  AngleAxisd(M_PI, Vector3d::UnitY()));

    calibrations.clear();
    cam_2_world.clear();
    for(int i=0;i<world_2_cam.size();++i){
        calibrations.push_back(calibration);
        cam_2_world.push_back(world_2_cam[i].inverse());
    }

    for(int i=0;i<100;++i) {
        double noise = 10.0/100.0 * i;

        double sum=0;
        const int test_size = 1000;
        for(int j=0;j<test_size;++j) {
            Vector3d p3_result = test_point(cam_2_world, calibrations, marker_pose.translation(), noise);
            Vector3d diff = p3_result - marker_pose.translation();
            sum += diff.norm()*1000;
        }


        SPDLOG_INFO("4 observations, noise of {0} pixel : error {1}mm", noise, sum/test_size);
    }


}