/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_TRACKINGCAMERA_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_TRACKINGCAMERA_H_

#include "traact/vision.h"

namespace traact::vision {

class TrackingCamera {
 public:
    void setData(const spatial::Pose6D &camera2world,
                 const CameraCalibration &calibration,
                 const KeyPointList &input);
    //std::vector<size_t> findPoints(const vision::Position3D world2point, traact::Scalar max_distance);

    std::vector<Eigen::ParametrizedLine<traact::Scalar , 3> > rays_;

    vision::KeyPointList input_;
    vision::CameraCalibration calibration_;
    spatial::Pose6D camera2world_;
};
} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_OUTSIDE_IN_TRACKINGCAMERA_H_
