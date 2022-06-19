/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "FindTargetPoints.h"
#include "traact/opencv/OpenCVUtils.h"

namespace traact::vision::outside_in {
std::optional<std::vector<int>>
FindTargetPoints::findTarget(const vision::Position3DList &current_points) {

    auto model_count = distances_model_.size();
    auto all_count = current_points.size();
    if (all_count < model_count)
        return {};

    std::vector<std::map<size_t, size_t> > final_points(model_count);

    distances_all_.resize(all_count);
    for (int i = 0; i < all_count; ++i) {
        distances_all_[i].resize(all_count);
        for (int j = 0; j < all_count; ++j) {
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
        std::vector<int> model_to_point(found_correspondences.size());
        for (const auto [model_index, point_index] : found_correspondences) {
            model_to_point[model_index] = point_index;
        }
        return {model_to_point};
    } else {
        return {};
    }
}

bool FindTargetPoints::TestPointAsOrigin(size_t origin_idx,
                                         std::map<size_t, size_t> &correspondences,
                                         const Position3DList &current_points) {
    correspondences.clear();

    if (!IsModelPoint(0, origin_idx))
        return false;

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
                                        [model_dist, max_distance = this->max_distance_](const std::pair<int, traact::Scalar> &value) {
                                            traact::Scalar diff = std::abs(value.second - model_dist);
                                            return diff < max_distance;
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

    return {};
}
void FindTargetPoints::initTarget(const Position3DList &model_points) {
    auto model_count = model_points.size();

    Scalar min_distance_model{std::numeric_limits<Scalar>::max()};

    distances_model_.resize(model_count);
    for (int i = 0; i < model_count; ++i) {
        distances_model_[i].resize(model_count);
        for (int j = 0; j < model_count; ++j) {
            distances_model_[i][j] = normL2Sqr(model_points[i] - model_points[j]);
            if(distances_model_[i][j] > 0.0){
                min_distance_model = std::min(min_distance_model, distances_model_[i][j]);
            }

        }
    }
    max_distance_ = min_distance_model / 2;

}

}