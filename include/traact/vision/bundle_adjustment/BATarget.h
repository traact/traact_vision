/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BATARGET_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BATARGET_H_

#include "traact/vision.h"
#include "BAData.h"


namespace traact::vision::bundle_adjustment {

class BATarget : public BAData<std::vector<Eigen::Vector3d>>{
 public:
    typedef typename std::shared_ptr<BATarget> Ptr;
    void SetTargetData(Position3DList model_points);
    std::vector<Eigen::Vector3d> GetTargetData();
    void SetStdDev(double stddev);
    double GetStdDev();
    void SetUseTargetResidual(bool value);
    bool IsUseTargetResidual();

    std::string toString();
    void SetMeasurement(Timestamp ts, Eigen::Affine3d pose);
 protected:
    std::vector<Eigen::Vector3d> model_;
    double residual_stddev_{1.0};
    bool use_target_residual_{false};


};

} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BATARGET_H_
