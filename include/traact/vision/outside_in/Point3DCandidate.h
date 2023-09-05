/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_POINT3DCANDIDATE_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_POINT3DCANDIDATE_H_

#include "traact/vision.h"

namespace traact::vision {

class Point3DCandidate {
 public:
    std::map<size_t, std::set<size_t> > candidate_points;
    vision::Position3D point;

    std::map<size_t, size_t> getGoodCandidates();
    void removeCandidate(size_t camera_idx, size_t point_idx);
    void addCandidate(size_t camera_idx, size_t point_idx);
    bool usesPoint(size_t camera_idx, size_t point_idx) const;
};

} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_POINT3DCANDIDATE_H_
