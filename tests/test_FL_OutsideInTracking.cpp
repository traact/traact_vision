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
#include <traact/util/Logging.h>
#include "spdlog/sinks/stdout_color_sinks.h"

#include <traact/math/perspective.h>
#include <traact/vision.h>
#include <traact/spatial.h>

#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <traact/vision/FLOutsideInTracking.h>
#include <traact/util/GenerateMultiCameraBATestData.h>
#include <traact/opencv/OpenCVUtils.h>

#define DEBUG_RENDER

#ifdef DEBUG_RENDER
#include <opencv2/opencv.hpp>
#endif

TEST(TraactVisionTestSuite, FL_OutsideInTracking_6Cameras_PerfectData) {
    using namespace traact;
    using namespace traact::spatial;
    using namespace traact::vision;
    std::size_t camera_count = 6;
    std::size_t mea_count = 1000;
    int width = 640;
    int height = 480;

    Position3DList model_points;
    model_points.push_back(spatial::Position3D(0,0,0));
//    model_points.push_back(spatial::Position3D(0.13252583341776236,0,0));
//    model_points.push_back(spatial::Position3D(0.34277024217893495, 0.12163979580583522,0));
//    model_points.push_back(spatial::Position3D(0.22012879651446983, 0.35336019453355811, 0.29034381052086565));
//    model_points.push_back(spatial::Position3D(-0.1359610986437321, 0.19759050230334861, 0.3067203013358311));
//    model_points.push_back(spatial::Position3D(0.029846395511918637,0.32073087182657503,0.29426261232679553));
    model_points.push_back(spatial::Position3D(0.2,0,0));
    model_points.push_back(spatial::Position3D(0.4,-0.05,0));
    model_points.push_back(spatial::Position3D(0,0,0.3));
    model_points.push_back(spatial::Position3D(0.15,0.1,0.3));
    model_points.push_back(spatial::Position3D(0.5,0,0.5));

    FLOutsideInTracking tracking;
    tracking.SetCountCameras(camera_count);
    util::GenerateMultiCameraBATestData test_data;
    test_data.Init(mea_count, camera_count, width, height, model_points, 0, 0, 0, 0);


    std::vector<spatial::Position2DList> output_points(camera_count);




#ifdef DEBUG_RENDER
    std::vector<cv::Scalar> camera_colors;
    camera_colors.push_back(cv::Scalar(60,0, 0));
    camera_colors.push_back(cv::Scalar(60,0, 64));
    camera_colors.push_back(cv::Scalar(60,0, 128));
    camera_colors.push_back(cv::Scalar(60,0, 256));
    camera_colors.push_back(cv::Scalar(60,128, 0));
    camera_colors.push_back(cv::Scalar(60,128, 64));



    std::vector<cv::Mat> debug_images(6);
    for (int i = 0; i < camera_count; ++i) {
        debug_images[i] = cv::Mat(height, width, CV_8UC3);
    }
#endif
    auto cam_data = test_data.GetCameraData();

    for (int mea_idx = 0; mea_idx < mea_count; ++mea_idx) {




        for (int i = 0; i < camera_count; ++i) {
            auto points2d = test_data.GetPointsForCamera(i);
            tracking.SetData(i, &cam_data[i].camera2world, &cam_data[i].calibration, &points2d);

        }
        tracking.Compute();

        spatial::Pose6D world2target_pose;
        //FLOutsideInTracking::TargetTrackingOutput target_points;
        bool result_valid = tracking.FindTarget(model_points, world2target_pose, nullptr, nullptr);

        EXPECT_TRUE(result_valid);
        if(result_valid){
            //spdlog::info("valid result {0} {1} {2}", world2target_pose.translation().x(), world2target_pose.translation().y(), world2target_pose.translation().z());
            spatial::Pose6D expected_pose = test_data.GetTargetPose(mea_idx);
            spatial::Pose6D diff_pose = world2target_pose * expected_pose.inverse();
            Eigen::Quaterniond diff_rot(diff_pose.rotation());
            diff_rot.normalize();
            EXPECT_NEAR(diff_pose.translation().norm(), 0, 0.001);
            EXPECT_NEAR(diff_rot.x(), 0, 0.001);
            EXPECT_NEAR(diff_rot.y(), 0, 0.001);
            EXPECT_NEAR(diff_rot.z(), 0, 0.001);

        }

//        for (int i = 0; i < target_points.size(); ++i) {
//            spdlog::info("target point {0}",i);
//            for (auto& camIdx_pointIdx : target_points[i]) {
//                spdlog::info("camera {0} point {1}", camIdx_pointIdx.first, camIdx_pointIdx.second);
//            }
//        }



#ifdef DEBUG_RENDER

        auto point3d_result = tracking.Get3DPoints();
        for (int i = 0; i < camera_count; ++i) {
            debug_images[i].setTo(0);
            auto points2d = test_data.GetPointsForCamera(i);

            for (int j = 0; j < points2d.size(); ++j) {
                cv::circle(debug_images[i], eigen2cv(points2d[j]), 7, cv::Scalar(255,0,0));
            }

//            for (int j = 0; j < camera_count; ++j) {
//                if(j == i)
//                    continue;
//                for(auto& ray : tracking.cameras_[j].rays_) {
//                    spatial::Position3D p1 = ray.pointAt(4);
//                    spatial::Position3D p2 = ray.pointAt(6);
//                    spatial::Position2D p2d1 = math::reproject_point(cam_data[i].camera2world, cam_data[i].calibration, p1);
//                    spatial::Position2D p2d2 = math::reproject_point(cam_data[i].camera2world, cam_data[i].calibration, p2);
//                    cv::line(debug_images[i], eigen2cv(p2d1),eigen2cv(p2d2),camera_colors[j]);
//                }
//            }

            for (int j = 0; j < point3d_result.size(); ++j) {
                spatial::Position2D p = math::reproject_point(cam_data[i].camera2world, cam_data[i].calibration, point3d_result[j]);
                cv::circle(debug_images[i], eigen2cv(p), 5, cv::Scalar(0,255,0));
            }

            if(result_valid) {
                spatial::Pose6D camera2target = cam_data[i].camera2world * world2target_pose;
                cv::Mat rvec, tvec;
                cv::Mat camera_matrix, distortion;
                traact2cv(cam_data[i].calibration, camera_matrix, distortion);
                traact2cv(camera2target, rvec,tvec);
                cv::drawFrameAxes(debug_images[i], camera_matrix, distortion, rvec, tvec, 1);
            }


        }

        for (int i = 0; i < camera_count; ++i) {
            cv::imshow(fmt::format("camera_{0:00}", i), debug_images[i]);
        }
        int key = cv::waitKey(10);
//        while(key != 'n'){
//            key = cv::waitKey(100);
//        }
#endif
        test_data.Next();
    }




}

TEST(TraactVisionTestSuite, FL_OutsideInTracking_6Cameras_Noise1Pixel) {
    using namespace traact;
    using namespace traact::spatial;
    using namespace traact::vision;
    std::size_t camera_count = 6;
    std::size_t mea_count = 1000;
    int width = 640;
    int height = 480;

    Position3DList model_points;
    model_points.push_back(spatial::Position3D(0,0,0));
//    model_points.push_back(spatial::Position3D(0.13252583341776236,0,0));
//    model_points.push_back(spatial::Position3D(0.34277024217893495, 0.12163979580583522,0));
//    model_points.push_back(spatial::Position3D(0.22012879651446983, 0.35336019453355811, 0.29034381052086565));
//    model_points.push_back(spatial::Position3D(-0.1359610986437321, 0.19759050230334861, 0.3067203013358311));
//    model_points.push_back(spatial::Position3D(0.029846395511918637,0.32073087182657503,0.29426261232679553));
    model_points.push_back(spatial::Position3D(0.2,0,0));
    model_points.push_back(spatial::Position3D(0.4,-0.05,0));
    model_points.push_back(spatial::Position3D(0,0,0.3));
    model_points.push_back(spatial::Position3D(0.15,0.1,0.3));
    model_points.push_back(spatial::Position3D(0.5,0,0.5));

    FLOutsideInTracking tracking;
    tracking.SetCountCameras(camera_count);
    util::GenerateMultiCameraBATestData test_data;
    test_data.Init(mea_count, camera_count, width, height, model_points, 0.20, 0, 0, 10);


    std::vector<spatial::Position2DList> output_points(camera_count);




#ifdef DEBUG_RENDER
    std::vector<cv::Scalar> camera_colors;
    camera_colors.push_back(cv::Scalar(60,0, 0));
    camera_colors.push_back(cv::Scalar(60,0, 64));
    camera_colors.push_back(cv::Scalar(60,0, 128));
    camera_colors.push_back(cv::Scalar(60,0, 256));
    camera_colors.push_back(cv::Scalar(60,128, 0));
    camera_colors.push_back(cv::Scalar(60,128, 64));

    std::vector<cv::Scalar> distinct_colors;
    distinct_colors.push_back(cv::Scalar(0,0,128));
    distinct_colors.push_back(cv::Scalar(40,110,40));
    distinct_colors.push_back(cv::Scalar(0,128,128));
    distinct_colors.push_back(cv::Scalar(128,128,0));
    distinct_colors.push_back(cv::Scalar(128,0,0));
    distinct_colors.push_back(cv::Scalar(75,25,230));
    distinct_colors.push_back(cv::Scalar(48,130,245));
    distinct_colors.push_back(cv::Scalar(25,225,225));
    distinct_colors.push_back(cv::Scalar(60,245,210));
    distinct_colors.push_back(cv::Scalar(75,180,60));
    distinct_colors.push_back(cv::Scalar(240,240,70));
    distinct_colors.push_back(cv::Scalar(200,130,0));
    distinct_colors.push_back(cv::Scalar(180,30,145));
    distinct_colors.push_back(cv::Scalar(230,50,240));
    distinct_colors.push_back(cv::Scalar(212,190,250));
    distinct_colors.push_back(cv::Scalar(200,250,255));

    distinct_colors.push_back(cv::Scalar(0,0,0));
    distinct_colors.push_back(cv::Scalar(0,0,0));

    std::vector<cv::Mat> debug_images(6);
    for (int i = 0; i < camera_count; ++i) {
        debug_images[i] = cv::Mat(height, width, CV_8UC3);
        cv::namedWindow(fmt::format("camera_{0:00}", i), cv::WINDOW_NORMAL);
        cv::resizeWindow(fmt::format("camera_{0:00}", i), width,height);
    }
#endif
    auto cam_data = test_data.GetCameraData();

    for (int mea_idx = 0; mea_idx < mea_count; ++mea_idx) {



        std::vector<spatial::Position2DList *> output_points;
        for (int i = 0; i < camera_count; ++i) {
            auto points2d = test_data.GetPointsForCameraNoise(mea_idx, i);
            tracking.SetData(i, &cam_data[i].camera2world, &cam_data[i].calibration, &points2d);
            output_points.push_back(new spatial::Position2DList);
        }
        tracking.Compute();
        auto reconstructed_points = tracking.Get3DPoints();

        for (int model_idx = 0; model_idx < model_points.size(); ++model_idx) {
            int found_count = 0;
            double distance = 0;
            Eigen::Vector3d model_world2point = test_data.GetTargetPose(mea_idx) * model_points[model_idx];
            for (int point_idx = 0; point_idx < reconstructed_points.size(); ++point_idx) {

                Eigen::Vector3d diff = model_world2point - reconstructed_points[point_idx];
                if(diff.norm() < 0.01) {
                    ++found_count;
                    distance = diff.norm();
                }

            }
            if(found_count == 0){
                spdlog::error("missing model point {0}", model_idx);
            } else if (found_count > 1) {
                spdlog::error("multiple points to close to model point {0}", model_idx);
            } else {
                spdlog::info("model point found {0}, distance {1}", model_idx, distance);
            }
        }

        spatial::Pose6D world2target_pose;
        //FLOutsideInTracking::TargetTrackingOutput target_points;


        bool result_valid = tracking.FindTarget(model_points, world2target_pose, &output_points, nullptr);

        EXPECT_TRUE(result_valid);
        if(result_valid){
            //spdlog::info("valid result {0} {1} {2}", world2target_pose.translation().x(), world2target_pose.translation().y(), world2target_pose.translation().z());
            spatial::Pose6D expected_pose = test_data.GetTargetPose(mea_idx);



//            spatial::Pose6D diff_pose = world2target_pose * expected_pose.inverse();
//            Eigen::Quaterniond diff_rot(diff_pose.rotation());
//            diff_rot.normalize();
//            EXPECT_NEAR(diff_pose.translation().norm(), 0, 0.02);
//            EXPECT_NEAR(diff_rot.x(), 0, 0.01);
//            EXPECT_NEAR(diff_rot.y(), 0, 0.01);
//            EXPECT_NEAR(diff_rot.z(), 0, 0.01);

        }

//        for (int i = 0; i < target_points.size(); ++i) {
//            spdlog::info("target point {0}",i);
//            for (auto& camIdx_pointIdx : target_points[i]) {
//                spdlog::info("camera {0} point {1}", camIdx_pointIdx.first, camIdx_pointIdx.second);
//            }
//        }



#ifdef DEBUG_RENDER

        auto point3d_result = tracking.Get3DPoints();
        for (int i = 0; i < camera_count; ++i) {
            debug_images[i].setTo(0);
            auto points2d = test_data.GetPointsForCameraNoise(mea_idx, i);

            for (int j = 0; j < points2d.size(); ++j) {
                cv::Scalar pointColor(255,0,0);
//                if(j < model_points.size()){
//                    pointColor = cv::Scalar(0,255,0);
//                }
                cv::circle(debug_images[i], eigen2cv(points2d[j]), 7, pointColor);
            }

//            for (int j = 0; j < camera_count; ++j) {
//                if(j == i)
//                    continue;
//                for(auto& ray : tracking.cameras_[j].rays_) {
//                    spatial::Position3D p1 = ray.pointAt(4);
//                    spatial::Position3D p2 = ray.pointAt(6);
//                    spatial::Position2D p2d1 = math::reproject_point(cam_data[i].camera2world, cam_data[i].calibration, p1);
//                    spatial::Position2D p2d2 = math::reproject_point(cam_data[i].camera2world, cam_data[i].calibration, p2);
//                    cv::line(debug_images[i], eigen2cv(p2d1),eigen2cv(p2d2),camera_colors[j]);
//                }
//            }

            for (int j = 0; j < point3d_result.size(); ++j) {
                double color_factor = static_cast<double>(255)/point3d_result.size();
                spatial::Position2D p = math::reproject_point(cam_data[i].camera2world, cam_data[i].calibration, point3d_result[j]);
                cv::circle(debug_images[i], eigen2cv(p), 5, distinct_colors[j],1);
            }

            if(result_valid) {
                spatial::Pose6D camera2target = cam_data[i].camera2world * world2target_pose;
                cv::Mat rvec, tvec;
                cv::Mat camera_matrix, distortion;
                traact2cv(cam_data[i].calibration, camera_matrix, distortion);
                traact2cv(camera2target, rvec,tvec);
                cv::drawFrameAxes(debug_images[i], camera_matrix, distortion, rvec, tvec, 1);

                for (int j = 0; j < output_points[i]->size(); ++j) {
                    cv::Point2d p = eigen2cv(output_points[i]->at(j));
                    cv::Point2d p_text(j*width/model_points.size(),30);
                    cv::circle(debug_images[i], p , 3, cv::Scalar(0,0,255));
                    //cv::line(debug_images[i], p, p_text, cv::Scalar(0,0,255),1);
                    cv::putText(debug_images[i], //target image
                                fmt::format("{0}", j), //text
                                p_text, //top-left position
                                cv::FONT_HERSHEY_DUPLEX,
                                1.0,
                                CV_RGB(0, 0, 255), //font color
                                2);
                }
            }


        }

        for (int i = 0; i < camera_count; ++i) {
            delete output_points[i];
        }

        for (int i = 0; i < camera_count; ++i) {
            cv::imshow(fmt::format("camera_{0:00}", i), debug_images[i]);
        }
        int key = cv::waitKey(10);
        while(key != 'n'){
            key = cv::waitKey(100);
        }
#endif
        test_data.Next();
    }




}