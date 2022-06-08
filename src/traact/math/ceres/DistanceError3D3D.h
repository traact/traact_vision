/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_DISTANCEERROR3D3D_H
#define TRAACTMULTI_DISTANCEERROR3D3D_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "traact/vision_datatypes.h"
#include <traact/spatial.h>

namespace traact::math {

template<uint16_t N>
struct DistanceError3D3D {
    // (u, v): the position of the observation with respect to the image
    // center point.
    DistanceError3D3D(spatial::Position3DList src_points, spatial::Position3DList dst_points)
        : src_points_(src_points), dst_points_(dst_points) {
    }

    template<typename T>
    bool operator()(const T *const target_pose,
                    T *residuals) const {
        // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
        //
        // We use QuaternionRotatePoint as it does not assume that the
        // quaternion is normalized, since one of the ways to run the
        // bundle adjuster is to let Ceres optimize all 4 quaternion
        // parameters without a local parameterization.

        const T *world2target_rot = target_pose;
        const T *world2target_pos = target_pose + 4;

        for (int i = 0; i < N; ++i) {
            T point_src[3];
            point_src[0] = T(src_points_[i].x());
            point_src[1] = T(src_points_[i].y());
            point_src[2] = T(src_points_[i].z());

            T point_dst[3];
            point_dst[0] = T(dst_points_[i].x());
            point_dst[1] = T(dst_points_[i].y());
            point_dst[2] = T(dst_points_[i].z());

            T point_predicted[3];
            ceres::QuaternionRotatePoint(world2target_rot, point_src, point_predicted);
            point_predicted[0] += world2target_pos[0];
            point_predicted[1] += world2target_pos[1];
            point_predicted[2] += world2target_pos[2];

            T dx = point_dst[0] - point_predicted[0];
            T dy = point_dst[1] - point_predicted[1];
            T dz = point_dst[2] - point_predicted[2];

            // The error is the difference between the predicted and observed position.
            residuals[i] = dx * dx + dy * dy + dz * dz;//sqrt(dx*dx+dy*dy+dz*dz);
        }

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(spatial::Position3DList src_points, spatial::Position3DList dst_points) {
        return (new ceres::AutoDiffCostFunction<
            DistanceError3D3D, N, 7>(
            new DistanceError3D3D(src_points, dst_points)));
    }

    const spatial::Position3DList src_points_;
    const spatial::Position3DList dst_points_;

};

class DistanceError3D3DFactory {
 public:
    static ceres::CostFunction *Create(spatial::Position3DList src_points, spatial::Position3DList dst_points) {

        size_t point_count = src_points.size();
        switch (point_count) {
            case 1:return DistanceError3D3D<1>::Create(src_points, dst_points);
            case 2:return DistanceError3D3D<2>::Create(src_points, dst_points);
            case 3:return DistanceError3D3D<3>::Create(src_points, dst_points);
            case 4:return DistanceError3D3D<4>::Create(src_points, dst_points);
            case 5:return DistanceError3D3D<5>::Create(src_points, dst_points);
            case 6:return DistanceError3D3D<6>::Create(src_points, dst_points);
            case 7:return DistanceError3D3D<7>::Create(src_points, dst_points);
            case 8:return DistanceError3D3D<8>::Create(src_points, dst_points);
            case 9:return DistanceError3D3D<9>::Create(src_points, dst_points);
            case 10:return DistanceError3D3D<10>::Create(src_points, dst_points);
            case 11:return DistanceError3D3D<11>::Create(src_points, dst_points);
            case 12:return DistanceError3D3D<12>::Create(src_points, dst_points);
            case 13:return DistanceError3D3D<13>::Create(src_points, dst_points);
            case 14:return DistanceError3D3D<14>::Create(src_points, dst_points);
            case 15:return DistanceError3D3D<15>::Create(src_points, dst_points);
            case 16:return DistanceError3D3D<16>::Create(src_points, dst_points);
            case 0:
            default:SPDLOG_ERROR("unsupported number of observations for CeresTargetNPointReprojectionError");
                return 0;

        }

    }
};
}

#endif //TRAACTMULTI_DISTANCEERROR3D3D_H
