/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/
#include <traact/opencv/OpenCVUtils.h>
#include "BATarget.h"

namespace traact::vision::bundle_adjustment {

void BATarget::SetTargetData(Position3DList model_points) {
    model_.reserve(model_points.size());
    for (const auto& point : model_points) {
        model_.emplace_back(point.x,point.y,point.z);
    }

}

std::vector<Eigen::Vector3d> BATarget::GetTargetData() {
    return model_;
}

std::string BATarget::toString() {
    std::stringstream ss;

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    ss << "Model Points: " << model_.size()<< std::endl;
    for(const auto& tmp : model_){
        ss << fmt::format("[{0}, {1}, {2}]",tmp.x(),tmp.y(),tmp.z()) << std::endl;
    }
    ss << "Measurements: " << data_.size();


    return ss.str();
}

void BATarget::SetStdDev(double stddev) {
    residual_stddev_ = stddev;

}

double BATarget::GetStdDev() {
    return residual_stddev_;
}

void BATarget::SetUseTargetResidual(bool value) {
    use_target_residual_ = value;
}

bool BATarget::IsUseTargetResidual() {
    return use_target_residual_;
}
void BATarget::SetMeasurement(Timestamp ts, Eigen::Affine3d pose) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& model_point : model_) {
        points.emplace_back(pose * model_point);
    }
    data_.emplace(ts, points);
}

} // traact