/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_POINTESTIMATION_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_POINTESTIMATION_H_

#include "traact/vision.h"
#include "TrackingCamera.h"

namespace traact::vision::outside_in {

void estimatePoints(const std::vector<const spatial::Pose6D*> &camera2world,
                    const std::vector<const CameraCalibration*> &calibration,
                    const std::vector<const KeyPointList*> &input,
                    vision::Position3DList &output_points, std::vector<std::map<size_t, size_t>> *output_matches);


} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_POINTESTIMATION_H_
