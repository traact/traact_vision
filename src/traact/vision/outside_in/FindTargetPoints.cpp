/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "FindTargetPoints.h"
#include "traact/opencv/OpenCVUtils.h"

namespace traact::vision::outside_in {
std::optional<std::vector<int>>
FindTargetPoints::FindTarget(const vision::Position3DList &model_points,
                             const vision::Position3DList &current_points) {

    auto model_count = model_points.size();
    auto all_count = current_points.size();
    if (all_count < model_count)
        return {};

    std::vector<std::map<size_t, size_t> > final_points(model_count);

    distances_model_.resize(model_count);
    for (int i = 0; i < model_count; ++i) {
        distances_model_[i].resize(model_count);
        for (int j = 0; j < model_count; ++j) {
            distances_model_[i][j] = normL2Sqr(model_points[i] - model_points[j]);
        }

    }
    distances_all_.resize(all_count);
    for (int i = 0; i < all_count; ++i) {
        distances_all_[i].resize(all_count);
        for (int j = 0; j < all_count; ++j) {
            //if(i == j)
            //    continue;
            //distances_all[i].push_back(std::make_pair(j, (current_points[i] - current_points[j]).squaredNorm()));
            distances_all_[i][j] = std::make_pair(j, normL2Sqr(current_points[i] - current_points[j]));
        }
    }
    std::map<size_t, size_t> found_correspondences;

    bool result = false;
    for (int point_idx = 0; point_idx < all_count; ++point_idx) {
        result = TestPointAsOrigin(point_idx, found_correspondences, current_points);
        if (result)
            break;
    }

    if (result) {

        //SPDLOG_INFO("found model: {0}", found_correspondences.size());



    }

}

bool FindTargetPoints::TestPointAsOrigin(size_t origin_idx,
                                         std::map<size_t, size_t> &correspondences,
                                         const Position3DList &current_points) {
    correspondences.clear();

    if (!IsModelPoint(0, origin_idx))
        return false;

    SPDLOG_INFO("model idx: {0} p {1} {2} {3} {4}",
                0,
                current_points[origin_idx].x,
                current_points[origin_idx].y,
                current_points[origin_idx].z,
                cv::norm(current_points[origin_idx]));

    correspondences[0] = origin_idx;

    correspondences = RecursiveFindModel(1, distances_model_.size(), correspondences, current_points);

    return correspondences.size() == distances_model_.size();
}

bool FindTargetPoints::IsModelPoint(size_t model_idx, size_t point_idx) {
    auto &model_distances = distances_model_[model_idx];
    auto &local_distances = distances_all_[point_idx];

    for (int i = 0; i < model_distances.size(); ++i) {
        if (i == model_idx)
            continue;
        traact::Scalar model_dist = model_distances[i];
        auto find_result = std::find_if(local_distances.begin(),
                                        local_distances.end(),
                                        [model_dist](const std::pair<int, traact::Scalar> &value) {
                                            traact::Scalar diff = std::abs(value.second - model_dist);
                                            return diff < 0.01;
                                        });
        if (find_result == local_distances.end())
            return false;
    }

    return true;
}

std::map<size_t, size_t>
FindTargetPoints::RecursiveFindModel(size_t cur_model_idx,
                                     size_t model_count,
                                     std::map<size_t, size_t> correspondences,
                                     const vision::Position3DList &current_points) {
    if (cur_model_idx >= model_count)
        return correspondences;

    for (int point_idx = 0; point_idx < current_points.size(); ++point_idx) {
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
        std::map<size_t, size_t> result = RecursiveFindModel(cur_model_idx + 1,
                                                             model_count,
                                                             next_test,
                                                             current_points);
        if (result.size() == model_count)
            return result;
    }

    return std::map<size_t, size_t>();
}

}