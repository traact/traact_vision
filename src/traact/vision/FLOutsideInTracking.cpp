/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "FLOutsideInTracking.h"
#include <spdlog/spdlog.h>
#include "traact/math/perspective.h"
void traact::vision::FLOutsideInTracking::SetCountCameras(size_t count) {
    cameras_.resize(count);
}

void traact::vision::FLOutsideInTracking::SetData(size_t idx, const spatial::Pose6D *camera2world,
                                                  const traact::vision::CameraCalibration *calibration,
                                                  const spatial::Position2DList *input) {
    cameras_[idx].SetData(camera2world, calibration, input);
}

void traact::vision::FLOutsideInTracking::Compute() {

    double distance_threshold = 0.01;
    std::vector<Point3DCandidate> candidates;
    for (int camera_idx = 0; camera_idx < cameras_.size(); ++camera_idx) {
        for (int point_idx = 0; point_idx < cameras_[camera_idx].input_.size(); ++point_idx) {

            auto is_already_used = std::find_if(candidates.begin(),
                                                candidates.end(),
                                                [camera_idx, point_idx](const Point3DCandidate &x) {
                                                    return x.UsesPoint(camera_idx,
                                                                       point_idx);
                                                });
            if (is_already_used != candidates.end())
                continue;

            Point3DCandidate current_candidate;
            current_candidate.AddCandidate(camera_idx, point_idx);

            auto &line1 = cameras_[camera_idx].rays_[point_idx];
            auto &p1 = cameras_[camera_idx].input_[point_idx];

            for (int camera_idx2 = camera_idx + 1; camera_idx2 < cameras_.size(); ++camera_idx2) {

                for (int point_idx2 = 0; point_idx2 < cameras_[camera_idx2].input_.size(); ++point_idx2) {
                    auto &line2 = cameras_[camera_idx2].rays_[point_idx2];
                    auto &p2 = cameras_[camera_idx2].input_[point_idx2];

                    Eigen::Vector3d n = line1.direction().cross(line2.direction());
                    if (n.isApprox(Eigen::Vector3d::Zero(), 1e-5))
                        continue;
                    n.normalize();
                    Eigen::Vector3d diff = line2.origin() - line1.origin();
                    double distance = std::abs(n.dot(diff));
                    //SPDLOG_INFO("Camera {0} Point {1} to {2} {3}, distance {4}", camera_idx, point_idx, camera_idx2, point_idx2, distance);
                    if (distance < distance_threshold) {
                        current_candidate.AddCandidate(camera_idx2, point_idx2);
                    }

                }
            }

            candidates.push_back(current_candidate);
        }

    }

    current_points3D.clear();
    current_points2D.clear();
    for (auto &candidate : candidates) {
        auto good_candidates = candidate.GetGoodCandidates();
        auto candidate_size = good_candidates.size();
        //SPDLOG_INFO("new candidate with good points {0}", candidate_size);
        if (candidate_size < 2)
            continue;

        std::vector<vision::CameraCalibration> calibrations(candidate_size);
        std::vector<spatial::Pose6D> cam2world_poses(candidate_size);
        //spatial::Position2DList image_points(candidate_size);
        std::vector<Eigen::Vector2d> image_points(candidate_size);
        spatial::Position3D result;
        int idx = 0;
        for (auto &tmp : good_candidates) {
            auto camera_idx = tmp.first;
            auto point_idx = tmp.second;
            calibrations[idx] = cameras_[camera_idx].calibration_;
            cam2world_poses[idx] = cameras_[camera_idx].camera2world_;
            image_points[idx] = cameras_[camera_idx].input_[point_idx];
            idx++;
        }
        //std::vector<double> covar(3*3);
        bool result_valid = traact::math::estimate_3d_point(result, cam2world_poses, calibrations, image_points);
        if (!result_valid)
            continue;

        for (int i = 0; i < cam2world_poses.size(); ++i) {
            double error = math::reprojection_error(cam2world_poses[i], image_points[i], calibrations[i], result);
            if (error > 5)
                result_valid = false;
        }
        if (!result_valid)
            continue;

//        double stddev = 0;
//        for (int i = 0; i < 3; ++i) {
//            //double tmp = std::sqrt(covar[i+i*3]);
//            stddev += covar[i+i*3];
//        }
//        stddev = std::sqrt(stddev);
//        SPDLOG_INFO("stddev {0}", stddev);
//        if(stddev == 0.0)
//            continue;
//        if(stddev > 0.1)
//            continue;
//        SPDLOG_INFO("add stddev {0}", stddev);

        //SPDLOG_INFO("reconstructed point {0} {1} {2}", result.x(), result.y(), result.z());
        current_points3D.push_back(result);
        current_points2D.push_back(good_candidates);

    }

}

traact::spatial::Position3DList traact::vision::FLOutsideInTracking::Get3DPoints() {
    return current_points3D;
}

std::vector<std::vector<traact::spatial::Position2D>> traact::vision::FLOutsideInTracking::Get3DPoints2DCorrespondence() {
    return std::vector<std::vector<spatial::Position2D>>();
}

bool
traact::vision::FLOutsideInTracking::FindTarget(const spatial::Position3DList &model_points, spatial::Pose6D &output,
                                                std::vector<spatial::Position2DList *> *output_points,
                                                spatial::Position3DList *output_target_points) {

    auto model_count = model_points.size();
    auto all_count = current_points3D.size();
    if (all_count < model_count)
        return false;

    std::vector<std::map<size_t, size_t> > final_points(model_count);

    distances_model_.resize(model_count);
    for (int i = 0; i < model_count; ++i) {
        distances_model_[i].resize(model_count);
        for (int j = 0; j < model_count; ++j) {
            distances_model_[i][j] = (model_points[i] - model_points[j]).squaredNorm();
        }

    }
    distances_all_.resize(all_count);
    for (int i = 0; i < all_count; ++i) {
        distances_all_[i].resize(all_count);
        for (int j = 0; j < all_count; ++j) {
            //if(i == j)
            //    continue;
            //distances_all[i].push_back(std::make_pair(j, (current_points3D[i] - current_points3D[j]).squaredNorm()));
            distances_all_[i][j] = std::make_pair(j, (current_points3D[i] - current_points3D[j]).squaredNorm());
        }
    }
    std::map<size_t, size_t> found_correspondences;

    bool result = false;
    for (int point_idx = 0; point_idx < all_count; ++point_idx) {
        result = TestPointAsOrigin(point_idx, found_correspondences);
        if (result)
            break;
    }

    if (result) {

        //SPDLOG_INFO("found model: {0}", found_correspondences.size());


        std::vector<Eigen::Affine3d> cam2world;
        std::vector<vision::CameraCalibration> intrinsics;
        std::vector<spatial::Position2DList> image_point;
        size_t local_camera_idx = 0;

        for (int camera_idx = 0; camera_idx < cameras_.size(); ++camera_idx) {
            bool use_camera = true;

            for (int model_idx = 0; model_idx < model_count; ++model_idx) {
                auto &cameras_to_points2d = current_points2D[found_correspondences[model_idx]];
                auto find_result = cameras_to_points2d.find(camera_idx);
                if (find_result != cameras_to_points2d.end()) {
                    final_points[model_idx][camera_idx] = find_result->second;
                } else {
                    auto find_points_result = cameras_[camera_idx].FindPoints(
                        current_points3D[found_correspondences[model_idx]], 5);

                    if (find_points_result.size() == 1) {
                        final_points[model_idx][camera_idx] = find_points_result[0];
                    } else {
                        SPDLOG_WARN("too many possible matches for point, reject camera {0}, model idx {1}",
                                     camera_idx,
                                     model_idx);
                        use_camera = false;
                    }
                }
            }

            if (use_camera) {
                cam2world.push_back(cameras_[camera_idx].camera2world_);
                intrinsics.push_back(cameras_[camera_idx].calibration_);
                spatial::Position2DList local_image_points(model_count);

                for (int point3d_idx = 0; point3d_idx < model_count; ++point3d_idx) {
                    auto local_point_idx = final_points[point3d_idx][camera_idx];
                    local_image_points[point3d_idx] = cameras_[camera_idx].input_[local_point_idx];
                }

                if (output_points) {
                    (*output_points->at(camera_idx)) = local_image_points;
                }

                image_point.push_back(local_image_points);
                ++local_camera_idx;
            } else {
                if (output_points)
                    output_points->at(camera_idx)->resize(0);
            }
        }

        bool result_valid = math::estimate_3d_pose(output, cam2world, intrinsics, image_point, model_points);

        if (result_valid) {
            if (output_target_points) {
                output_target_points->resize(model_count);
                for (int model_idx = 0; model_idx < model_count; ++model_idx) {
                    output_target_points->at(model_idx) = current_points3D[found_correspondences[model_idx]];
                }

            }

            return true;
        }

        if (output_target_points)
            output_target_points->resize(0);

        return false;
    }

}

bool traact::vision::FLOutsideInTracking::TestPointAsOrigin(size_t origin_idx,
                                                            std::map<size_t, size_t> &correspondences) {
    correspondences.clear();

    if (!IsModelPoint(0, origin_idx))
        return false;

    SPDLOG_INFO("model idx: {0} p {1} {2} {3} {4}",
                 0,
                 current_points3D[origin_idx].x(),
                 current_points3D[origin_idx].y(),
                 current_points3D[origin_idx].z(),
                 current_points3D[origin_idx].norm());

    correspondences[0] = origin_idx;

    correspondences = RecursiveFindModel(1, distances_model_.size(), correspondences);

    return correspondences.size() == distances_model_.size();
}

bool traact::vision::FLOutsideInTracking::IsModelPoint(size_t model_idx, size_t point_idx) {
    auto &model_distances = distances_model_[model_idx];
    auto &local_distances = distances_all_[point_idx];

    for (int i = 0; i < model_distances.size(); ++i) {
        if (i == model_idx)
            continue;
        double model_dist = model_distances[i];
        auto find_result = std::find_if(local_distances.begin(),
                                        local_distances.end(),
                                        [model_dist](const std::pair<int, double> &value) {
                                            double diff = std::abs(value.second - model_dist);
                                            return diff < 0.01;
                                        });
        if (find_result == local_distances.end())
            return false;
    }

    return true;
}

std::map<size_t, size_t>
traact::vision::FLOutsideInTracking::RecursiveFindModel(size_t cur_model_idx, size_t model_count,
                                                        std::map<size_t, size_t> correspondences) {
    if (cur_model_idx >= model_count)
        return correspondences;

    for (int point_idx = 0; point_idx < current_points3D.size(); ++point_idx) {
        auto is_used = std::find_if(correspondences.begin(),
                                    correspondences.end(),
                                    [point_idx](const std::pair<size_t, size_t> &value) {
                                        return value.second == point_idx;
                                    });
        if (is_used != correspondences.end())
            continue;
        std::map<size_t, size_t> next_test = correspondences;
        if (!IsModelPoint(cur_model_idx, point_idx))
            continue;
        next_test[cur_model_idx] = point_idx;
        std::map<size_t, size_t> result = RecursiveFindModel(cur_model_idx + 1, model_count, next_test);
        if (result.size() == model_count)
            return result;
    }

    return std::map<size_t, size_t>();
}

void traact::vision::TrackingCamera::SetData(const traact::spatial::Pose6D *camera2world,
                                             const traact::vision::CameraCalibration *calibration,
                                             const traact::spatial::Position2DList *input) {

    camera2world_ = *camera2world;
    calibration_ = *calibration;
    input_ = *input;

    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();

    intrinsics(0, 0) = calibration->fx;
    intrinsics(1, 1) = calibration->fy;

    intrinsics(0, 1) = calibration->skew;

    intrinsics(0, 2) = calibration->cx;
    intrinsics(1, 2) = calibration->cy;

    Eigen::Matrix3d intrinsics_inv = intrinsics.inverse();

    rays_.resize(input->size());

    spatial::Pose6D world2camera = camera2world->inverse();

    for (int i = 0; i < input->size(); ++i) {
        Eigen::Vector3d p(input->at(i).x(), input->at(i).y(), 1);
        Eigen::Vector3d direction = intrinsics_inv * p;
        direction.normalize();
        direction = world2camera * direction;
        direction = direction - world2camera.translation();
        direction.normalize();
        rays_[i] = Eigen::ParametrizedLine<double, 3>(world2camera.translation(), direction);

    }

}

std::vector<size_t>
traact::vision::TrackingCamera::FindPoints(const traact::spatial::Position3D world2point, double max_distance) {
    std::vector<size_t> result;
    spatial::Position2D image_point = math::reproject_point(camera2world_, calibration_, world2point);
    double max_distance_squared = max_distance * max_distance;
    for (int i = 0; i < input_.size(); ++i) {
        double distance = (input_[i] - image_point).squaredNorm();
        if (distance < max_distance_squared)
            result.push_back(i);
    }
    return result;
}

std::map<size_t, size_t> traact::vision::Point3DCandidate::GetGoodCandidates() {
    std::map<size_t, size_t> result;
    for (auto &tmp : candidate_points) {
        if (tmp.second.size() == 1) {
            result.emplace(tmp.first, *tmp.second.begin());
        }
    }
    return result;
}

void traact::vision::Point3DCandidate::RemoveCandidate(size_t camera_idx, size_t point_idx) {
    auto result = candidate_points.find(camera_idx);
    if (result != candidate_points.end()) {
        candidate_points.at(camera_idx).erase(point_idx);
    }
}

void traact::vision::Point3DCandidate::AddCandidate(size_t camera_idx, size_t point_idx) {
    candidate_points[camera_idx].emplace(point_idx);
}

bool traact::vision::Point3DCandidate::UsesPoint(size_t camera_idx, size_t point_idx) const {
    auto result = candidate_points.find(camera_idx);
    if (result != candidate_points.end()) {
        if (result->second.count(point_idx) > 0)
            return true;
    }
    return false;
}
