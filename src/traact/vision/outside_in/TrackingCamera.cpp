/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "TrackingCamera.h"
#include "traact/math/perspective.h"
#include "traact/opencv/OpenCVUtils.h"

namespace traact::vision {
void traact::vision::TrackingCamera::setData(const spatial::Pose6D &camera2world,
                                             const CameraCalibration &calibration,
                                             const KeyPointList &input) {

    input_ = input;
    calibration_ = calibration;
    camera2world_ = camera2world;

    Eigen::Matrix3<traact::Scalar> intrinsics = Eigen::Matrix3<traact::Scalar>::Identity();

    intrinsics(0, 0) = calibration.fx;
    intrinsics(1, 1) = calibration.fy;

    intrinsics(0, 1) = calibration.skew;

    intrinsics(0, 2) = calibration.cx;
    intrinsics(1, 2) = calibration.cy;

    Eigen::Matrix3<traact::Scalar> intrinsics_inv = intrinsics.inverse();

    rays_.resize(input.size());

    spatial::Pose6D world2camera = camera2world.inverse();

    for (size_t i = 0; i < input.size(); ++i) {
        Eigen::Vector3<traact::Scalar> p(input.at(i).pt.x, input.at(i).pt.y, 1);
        Eigen::Vector3<traact::Scalar> direction = intrinsics_inv * p;
        direction.normalize();
        direction = world2camera * direction;
        direction = direction - world2camera.translation();
        direction.normalize();
        rays_[i] = Eigen::ParametrizedLine<traact::Scalar , 3>(world2camera.translation(), direction);

    }

}

//std::vector<size_t>
//traact::vision::TrackingCamera::findPoints(const traact::vision::Position3D world2point, traact::Scalar max_distance) {
//    std::vector<size_t> result;
//    vision::Position2D image_point = math::reproject_point(camera2world_, calibration_, world2point);
//    traact::Scalar max_distance_squared = max_distance * max_distance;
//    for (size_t i = 0; i < input_.size(); ++i) {
//        traact::Scalar distance = traact::normL2Sqr(input_[i] - image_point);
//        if (distance < max_distance_squared)
//            result.push_back(i);
//    }
//    return result;
//}

} // traact