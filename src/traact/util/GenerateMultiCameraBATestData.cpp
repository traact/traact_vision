/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "traact/util/GenerateMultiCameraBATestData.h"
#include "traact/math/perspective.h"
#include <spdlog/spdlog.h>
#include <random>

void
traact::util::GenerateMultiCameraBATestData::Init(size_t data_count, size_t camera_count, size_t width,
                                                  size_t height,
                                                  vision::Position3DList model_points, traact::Scalar point2d_noise,
                                                  traact::Scalar camera_pos_noise, traact::Scalar camera_rot_noise,
                                                  size_t noise_point_count) {
    max_count_ = data_count;
    model_points_ = model_points;
    camera_data_.resize(camera_count);
    point2d_noise_ = point2d_noise;
    camera_pos_noise_ = camera_pos_noise;
    camera_rot_noise_ = camera_rot_noise;
    traact::Scalar distance = 3;

    Eigen::Vector3<traact::Scalar> target(0, 0, 0);
    Eigen::Vector3<traact::Scalar> up(0, -1, 0);
    Eigen::Vector3<traact::Scalar> position(distance, -distance, 0);
    Eigen::Matrix3<traact::Scalar> R;
    //R.col(2) = (position-target).normalized();
    R.col(2) = (target - position).normalized();
    R.col(0) = up.cross(R.col(2)).normalized();
    R.col(1) = R.col(2).cross(R.col(0));
    Eigen::Matrix4<traact::Scalar> view_matrix;
    view_matrix.topLeftCorner<3, 3>() = R.transpose();
    view_matrix.topRightCorner<3, 1>() = -R.transpose() * position;
    //view_matrix.topRightCorner<3,1>() = R.transpose() * position;
    view_matrix.row(3) << 0, 0, 0, 1;
    traact::spatial::Pose6D init_pose(view_matrix);
    init_pose = init_pose.inverse();
    Eigen::AngleAxis<traact::Scalar> next_rot(2 * M_PI / camera_count, up);

    unsigned long seed = 0;
    std::mt19937 gen(seed);
    std::normal_distribution<> n_dist(0, 1);
    std::uniform_real_distribution<> u_dist(0.1, 0.9);
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

        Eigen::Vector3<traact::Scalar> pos_noise
            (camera_pos_noise_ * n_dist(gen), camera_pos_noise_ * n_dist(gen), camera_pos_noise_ * n_dist(gen));
        traact::spatial::Rotation3D rot_noise
            (1.0, camera_pos_noise_ * n_dist(gen), camera_pos_noise_ * n_dist(gen), camera_pos_noise_ * n_dist(gen));
        rot_noise.normalize();
        traact::spatial::Pose6D pose_noise;
        pose_noise.fromPositionOrientationScale(pos_noise, rot_noise, Eigen::Vector3<traact::Scalar>::Ones());
        camera_data_[i].camera2world_noise = camera_data_[i].camera2world * pose_noise;
        init_pose.prerotate(next_rot);

    }

    expected_pose_.resize(data_count);
    reference_point2d_.resize(data_count);
    reference_point2d_noise_.resize(data_count);
    for (int i = 0; i < data_count; ++i) {
        spatial::Pose6D current_pose;
        current_pose.setIdentity();
        traact::Scalar r = i / static_cast<traact::Scalar>(data_count);
        traact::Scalar px = std::sin(r * M_PI * 2);
        traact::Scalar py = std::cos(r * M_PI * 2);
        traact::Scalar pz = std::sin(-r * M_PI * 2);

        vision::Position3D pos(px, py, pz);
        traact::spatial::Rotation3D rot(1.0 - r, r, 0, 0);
        rot.normalize();
        current_pose.translate(Eigen::Vector3<Scalar>(pos.x,pos.y,pos.z));
        current_pose.rotate(rot);

        expected_pose_[i] = current_pose;

        reference_point2d_[i].resize(camera_count);
        reference_point2d_noise_[i].resize(camera_count);
        for (int camera_idx = 0; camera_idx < camera_count; ++camera_idx) {
            traact::vision::Position2DList result(model_points_.size());
            traact::vision::Position2DList result_noise(model_points_.size() + noise_point_count);
            auto &cam_data = camera_data_[camera_idx];
            for (int model_idx = 0; model_idx < model_points_.size(); ++model_idx) {
                spatial::Pose6D camera2target = cam_data.camera2world * expected_pose_[i];
                result[model_idx] =
                    traact::math::reproject_point(camera2target, cam_data.calibration, model_points_[model_idx]);
                result_noise[model_idx] =
                    result[model_idx] + vision::Position2D(point2d_noise_ * n_dist(gen), point2d_noise_ * n_dist(gen));
            }
            reference_point2d_[i][camera_idx] = result;

            if (noise_point_count > 0) {
                for (int noise_point_idx = 0; noise_point_idx < noise_point_count; ++noise_point_idx) {
                    result_noise[model_points_.size() + noise_point_idx] =
                        vision::Position2D(width * u_dist(gen), height * u_dist(gen));
                }
            }
            std::mt19937 gen{0};
            std::shuffle(result_noise.begin(), result_noise.end(), gen);
            reference_point2d_noise_[i][camera_idx] = result_noise;
        }

    }
}

std::vector<traact::util::GenerateMultiCameraBATestDataCamera> traact::util::GenerateMultiCameraBATestData::GetCameraData() {
    return camera_data_;
}

traact::spatial::Pose6D traact::util::GenerateMultiCameraBATestData::GetTargetPose(size_t data_idx) {
    return expected_pose_[current_count_];
}

traact::vision::Position2DList traact::util::GenerateMultiCameraBATestData::GetPointsForCamera(size_t idx) {

    return reference_point2d_[current_count_][idx];
}

bool traact::util::GenerateMultiCameraBATestData::Next() {
    current_count_++;
    if (current_count_ >= max_count_)
        return false;

    return true;
}

traact::vision::Position2DList
traact::util::GenerateMultiCameraBATestData::GetPointsForCamera(size_t data_idx, size_t camera_idx) {
    return reference_point2d_[data_idx][camera_idx];
}

traact::vision::Position2DList
traact::util::GenerateMultiCameraBATestData::GetPointsForCameraNoise(size_t data_idx, size_t camera_idx) {
    return reference_point2d_noise_[data_idx][camera_idx];
}
