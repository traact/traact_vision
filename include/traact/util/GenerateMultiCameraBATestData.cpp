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

#include "GenerateMultiCameraBATestData.h"
#include <traact/math/perspective.h>
#include <spdlog/spdlog.h>
#include <random>

void
traact::util::GenerateMultiCameraBATestData::Init(std::size_t data_count, std::size_t camera_count, std::size_t width,
                                                  std::size_t height,
                                                  spatial::Position3DList model_points, double point2d_noise,
                                                  double camera_pos_noise, double camera_rot_noise,
                                                  std::size_t noise_point_count) {
    max_count_ = data_count;
    model_points_ = model_points;
    camera_data_.resize(camera_count);
    point2d_noise_ = point2d_noise;
    camera_pos_noise_ = camera_pos_noise;
    camera_rot_noise_ = camera_rot_noise;
    double distance = 3;

    Eigen::Vector3d target(0,0,0);
    Eigen::Vector3d up(0,-1,0);
    Eigen::Vector3d position(distance,-distance,0);
    Eigen::Matrix3d R;
    //R.col(2) = (position-target).normalized();
    R.col(2) = (target-position).normalized();
    R.col(0) = up.cross(R.col(2)).normalized();
    R.col(1) = R.col(2).cross(R.col(0));
    Eigen::Matrix4d view_matrix;
    view_matrix.topLeftCorner<3,3>() = R.transpose();
    view_matrix.topRightCorner<3,1>() = -R.transpose() * position;
    //view_matrix.topRightCorner<3,1>() = R.transpose() * position;
    view_matrix.row(3) << 0, 0, 0, 1;
    Eigen::Affine3d init_pose(view_matrix);
    init_pose = init_pose.inverse();
    Eigen::AngleAxisd next_rot(2*M_PI/camera_count, up);


    unsigned long seed = 0;
    std::mt19937 gen(seed);
    std::normal_distribution<> n_dist(0,1);
    std::uniform_real_distribution<> u_dist(0.1,0.9);
    for (int i = 0; i < camera_count; ++i) {
        camera_data_[i].calibration.radial_distortion.resize(0);
        camera_data_[i].calibration.tangential_distortion.resize(0);
        camera_data_[i].calibration.skew = 0;
        camera_data_[i].calibration.fx = width / 2;
        camera_data_[i].calibration.fy = width / 2;
        camera_data_[i].calibration.cx = width / 2;
        camera_data_[i].calibration.cy = height / 2;
        camera_data_[i].calibration.width = width;
        camera_data_[i].calibration.height = height;

        camera_data_[i].camera2world = init_pose.inverse();

        Eigen::Vector3d pos_noise(camera_pos_noise_*n_dist(gen),camera_pos_noise_*n_dist(gen),camera_pos_noise_*n_dist(gen));
        Eigen::Quaterniond rot_noise(1.0,camera_pos_noise_*n_dist(gen),camera_pos_noise_*n_dist(gen),camera_pos_noise_*n_dist(gen));
        rot_noise.normalize();
        Eigen::Affine3d pose_noise;
        pose_noise.fromPositionOrientationScale(pos_noise, rot_noise, Eigen::Vector3d::Ones());
        camera_data_[i].camera2world_noise = camera_data_[i].camera2world * pose_noise;
        init_pose.prerotate(next_rot);



    }


    expected_pose_.resize(data_count);
    reference_point2d_.resize(data_count);
    reference_point2d_noise_.resize(data_count);
    for (int i = 0; i < data_count; ++i) {
        spatial::Pose6D current_pose;
        current_pose.setIdentity();
        double r = i / static_cast<double>(data_count);
        double px = std::sin(r*M_PI*2);
        double py = std::cos(r*M_PI*2);
        double pz = std::sin(-r*M_PI*2);

        spatial::Position3D pos(px,py,pz);
        Eigen::Quaterniond rot(1.0 - r,r,0,0);
        rot.normalize();
        current_pose.translate(pos);
        current_pose.rotate(rot);

        expected_pose_[i] = current_pose;

        reference_point2d_[i].resize(camera_count);
        reference_point2d_noise_[i].resize(camera_count);
        for (int camera_idx = 0; camera_idx < camera_count; ++camera_idx) {
            traact::spatial::Position2DList result(model_points_.size());
            traact::spatial::Position2DList result_noise(model_points_.size()+noise_point_count);
            auto& cam_data = camera_data_[camera_idx];
            for(int model_idx=0;model_idx<model_points_.size();++model_idx) {
                spatial::Pose6D camera2target = cam_data.camera2world * expected_pose_[i];
                result[model_idx] = traact::math::reproject_point(camera2target, cam_data.calibration, model_points_[model_idx]);
                result_noise[model_idx] = result[model_idx] + spatial::Position2D(point2d_noise_ * n_dist(gen), point2d_noise_ * n_dist(gen));
            }
            reference_point2d_[i][camera_idx] = result;

            if(noise_point_count > 0) {
                for (int noise_point_idx = 0; noise_point_idx < noise_point_count; ++noise_point_idx) {
                    result_noise[model_points_.size()+noise_point_idx] = spatial::Position2D(width * u_dist(gen), height * u_dist(gen));
                }
            }
            std::mt19937 gen{0};
            std::shuffle(result_noise.begin(),result_noise.end(),gen);
            reference_point2d_noise_[i][camera_idx] = result_noise;
        }


    }
}

std::vector<traact::util::GenerateMultiCameraBATestDataCamera> traact::util::GenerateMultiCameraBATestData::GetCameraData() {
    return camera_data_;
}

traact::spatial::Pose6D traact::util::GenerateMultiCameraBATestData::GetTargetPose(std::size_t data_idx) {
    return expected_pose_[current_count_];
}

traact::spatial::Position2DList traact::util::GenerateMultiCameraBATestData::GetPointsForCamera(std::size_t idx) {

    return reference_point2d_[current_count_][idx];
}

bool traact::util::GenerateMultiCameraBATestData::Next() {
    current_count_++;
    if(current_count_ >= max_count_)
        return false;




    return true;
}

traact::spatial::Position2DList
traact::util::GenerateMultiCameraBATestData::GetPointsForCamera(std::size_t data_idx, std::size_t camera_idx) {
    return reference_point2d_[data_idx][camera_idx];
}

traact::spatial::Position2DList
traact::util::GenerateMultiCameraBATestData::GetPointsForCameraNoise(std::size_t data_idx, std::size_t camera_idx) {
    return reference_point2d_noise_[data_idx][camera_idx];
}
