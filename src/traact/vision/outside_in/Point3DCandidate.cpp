/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "Point3DCandidate.h"

namespace traact::vision {
std::map<size_t, size_t> traact::vision::Point3DCandidate::getGoodCandidates() {
    std::map<size_t, size_t> result;
    for (auto &tmp : candidate_points) {
        if (tmp.second.size() == 1) {
            result.emplace(tmp.first, *tmp.second.begin());
        }
    }
    return result;
}

void traact::vision::Point3DCandidate::removeCandidate(size_t camera_idx, size_t point_idx) {
    auto result = candidate_points.find(camera_idx);
    if (result != candidate_points.end()) {
        candidate_points.at(camera_idx).erase(point_idx);
    }
}

void traact::vision::Point3DCandidate::addCandidate(size_t camera_idx, size_t point_idx) {
    candidate_points[camera_idx].emplace(point_idx);
}

bool traact::vision::Point3DCandidate::usesPoint(size_t camera_idx, size_t point_idx) const {
    auto result = candidate_points.find(camera_idx);
    if (result != candidate_points.end()) {
        if (result->second.count(point_idx) > 0)
            return true;
    }
    return false;
}
} // traact