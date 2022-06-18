/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_FINDTARGETPOINTS_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_FINDTARGETPOINTS_H_

#include "traact/vision.h"

namespace traact::vision::outside_in {
class FindTargetPoints {
    std::optional<std::vector<int>> FindTarget(const vision::Position3DList &model_points,
                                               const vision::Position3DList &current_points);
 private:
    bool TestPointAsOrigin(size_t origin_idx,
                           std::map<size_t, size_t> &correspondences,
                           const Position3DList &current_points);
    bool IsModelPoint(size_t model_idx, size_t point_idx);
    std::map<size_t, size_t> RecursiveFindModel(size_t cur_model_idx,
                                                size_t model_count,
                                                std::map<size_t, size_t> correspondences,
                                                const vision::Position3DList &current_points);
    std::vector<std::vector<traact::Scalar> > distances_model_;
    std::vector<std::vector<std::pair<int, traact::Scalar> > > distances_all_;
};
}

#endif //TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_FINDTARGETPOINTS_H_
