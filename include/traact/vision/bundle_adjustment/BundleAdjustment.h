/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BUNDLEADJUSTMENT_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BUNDLEADJUSTMENT_H_

#include "traact/vision.h"
#include <ceres/ceres.h>
#include "BACamera.h"
#include "BATarget.h"

namespace traact::vision::bundle_adjustment {

class BundleAdjustment {
 public:
    ~BundleAdjustment();
    void AddCamera(BACamera::Ptr camera);
    void SetTarget(BATarget::Ptr target);

    bool CheckData();

    bool Optimize();

    std::vector<spatial::Pose6D> getResult();
 protected:
    std::vector<BACamera::Ptr> cameras_;
    BATarget::Ptr target_;
    std::vector<Timestamp> used_ts_;
    std::size_t GetCameraMeaCount(Timestamp ts);
    std::shared_ptr<ceres::Problem> ceres_problem_;
    double *ceres_parameter_;


    std::size_t target_points_count_{6};
    const std::size_t camera_parameter_size_{7};
    std::size_t target_parameter_size_{target_points_count_ * 3};
    double* GetCameraParameter(std::size_t idx);
    double* GetTargetParameter(std::size_t idx);
};

} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BUNDLEADJUSTMENT_H_
