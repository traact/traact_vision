/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "BACamera.h"
#include <traact/math/perspective.h>
namespace traact::vision::bundle_adjustment {
bool BACamera::isStaticPosition() const {
    return static_position;
}

void BACamera::setStaticPosition(bool staticPosition) {
    static_position = staticPosition;
}

bool BACamera::isStaticRotation() const {
    return static_rotation;
}

void BACamera::setStaticRotation(bool staticRotation) {
    static_rotation = staticRotation;
}

const traact::vision::CameraCalibration &BACamera::getIntrinsic() const {
    return intrinsic_;
}

void BACamera::setIntrinsic(const traact::vision::CameraCalibration &intrinsic) {
    intrinsic_ = intrinsic;
}

const traact::spatial::Pose6D &BACamera::getExtrinsic() const {
    return extrinsic_;
}

void BACamera::setExtrinsic(const traact::spatial::Pose6D &extrinsic) {
    extrinsic_ = extrinsic;
}

const std::string &BACamera::getResultfile() const {
    return resultfile_;
}

void BACamera::setResultfile(const std::string &resultfile) {
    resultfile_ = resultfile;
}

std::string BACamera::toString() {
    std::stringstream ss;

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    ss << "Name: " << name_ << std::endl;
    ss << "Static Position: " << static_position << std::endl;
    ss << "Static Rotation: " << static_rotation << std::endl;
    ss << "Intrinsic: " << intrinsic_ << std::endl;
    ss << "Extrinsic: " << std::endl << extrinsic_.matrix().format(CleanFmt) << std::endl;

    ss << "Resultfile: " << resultfile_ << std::endl;
    ss << "Measurements: " << data_.size();

    return ss.str();
}

BACamera::BACamera(const std::string &name) : name_(name) {

}
void BACamera::tryFindingImagePoints(const std::vector<Eigen::Vector3d> &world_points,
                                     std::vector<Eigen::Vector2d> &image_points,
                                     const KeyPointList &all_points) {

    //traact::math::reproject_point()

}

}