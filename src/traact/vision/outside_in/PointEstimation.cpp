/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "traact/vision/outside_in/PointEstimation.h"
#include "traact/vision/outside_in/Point3DCandidate.h"
#include "traact/math/perspective.h"

namespace traact::vision::outside_in {



void estimatePoints(const std::vector<const spatial::Pose6D *> &camera2world,
                    const std::vector<const CameraCalibration *> &calibration,
                    const std::vector<const KeyPointList *> &input,
                    Position3DList &output_points,
                    std::vector<std::map<size_t, size_t>> *output_matches, EstimatePointsParameter parameter) {

    if(camera2world.size() != calibration.size() || input.empty()){
        return;
    }

    std::vector<TrackingCamera> cameras;
    cameras.resize(camera2world.size());
    for (size_t i = 0; i < camera2world.size(); ++i) {
        cameras[i].setData(*camera2world[i], *calibration[i], *input[i]);
    }

    std::vector<Point3DCandidate> candidates;
    for (int camera_idx = 0; camera_idx < cameras.size(); ++camera_idx) {
        for (int point_idx = 0; point_idx < cameras[camera_idx].input_.size(); ++point_idx) {

            auto is_already_used = std::find_if(candidates.begin(),
                                                candidates.end(),
                                                [camera_idx, point_idx](const Point3DCandidate &x) {
                                                    return x.usesPoint(camera_idx,
                                                                       point_idx);
                                                });
            if (is_already_used != candidates.end())
                continue;

            Point3DCandidate current_candidate;
            current_candidate.addCandidate(camera_idx, point_idx);

            auto &line_1 = cameras[camera_idx].rays_[point_idx];
            auto &p_1 = cameras[camera_idx].input_[point_idx];

            for (int camera_idx_2 = camera_idx + 1; camera_idx_2 < cameras.size(); ++camera_idx_2) {

                for (int point_idx_2 = 0; point_idx_2 < cameras[camera_idx_2].input_.size(); ++point_idx_2) {
                    auto &line_2 = cameras[camera_idx_2].rays_[point_idx_2];
                    auto &p_2 = cameras[camera_idx_2].input_[point_idx_2].pt;

                    Eigen::Vector3<traact::Scalar> n = line_1.direction().cross(line_2.direction());
                    if (n.isApprox(Eigen::Vector3<traact::Scalar>::Zero(), 1e-5)) {
                        continue;
                    }
                    n.normalize();
                    Eigen::Vector3<traact::Scalar> diff = line_2.origin() - line_1.origin();
                    traact::Scalar distance = std::abs(n.dot(diff));
                    //SPDLOG_INFO("Camera {0} Point {1} to {2} {3}, distance {4}", camera_idx, point_idx, camera_idx2, point_idx2, distance);
                    if (distance < parameter.distance_threshold) {
                        current_candidate.addCandidate(camera_idx_2, point_idx_2);
                    }

                }
            }

            candidates.push_back(current_candidate);
        }

    }


    for (auto &candidate : candidates) {
        auto good_candidates = candidate.getGoodCandidates();
        auto candidate_size = good_candidates.size();
        //SPDLOG_INFO("new candidate with good points {0}", candidate_size);
        if (candidate_size < parameter.min_candidates)
            continue;

        std::vector<vision::CameraCalibration> calibrations(candidate_size);
        std::vector<spatial::Pose6D> cam2world_poses(candidate_size);
        //vision::Position2DList image_points(candidate_size);
        vision::Position2DList image_points(candidate_size);
        vision::Position3D result;
        int idx = 0;
        for (auto &tmp : good_candidates) {
            auto camera_idx = tmp.first;
            auto point_idx = tmp.second;
            calibrations[idx] = cameras[camera_idx].calibration_;
            cam2world_poses[idx] = cameras[camera_idx].camera2world_;
            image_points[idx] = cameras[camera_idx].input_[point_idx].pt;
            idx++;
        }
        //std::vector<traact::Scalar> covar(3*3);
        bool result_valid = traact::math::estimate_3d_point(result, cam2world_poses, calibrations, image_points);
        if (!result_valid)
            continue;

        for (int i = 0; i < cam2world_poses.size(); ++i) {
            traact::Scalar
                error = math::reprojectionError(cam2world_poses[i], image_points[i], calibrations[i], result);
            if (error > parameter.max_error)
                result_valid = false;
        }
        if (!result_valid)
            continue;

        //SPDLOG_INFO("reconstructed point {0} {1} {2}", result.x(), result.y(), result.z());

        if(output_matches){
            output_matches->emplace_back(good_candidates);
        }
        output_points.emplace_back(result);
    }
}
} // traact