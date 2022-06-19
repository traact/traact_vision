/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TARGETREPROJECTIONERROR_H
#define TRAACTMULTI_TARGETREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "traact/vision_datatypes.h"
#include <traact/spatial.h>

namespace traact::math {

template<uint16_t N>
struct TargetReprojectionError {
    // (u, v): the position of the observation with respect to the image
    // center point.
    TargetReprojectionError(vision::Position2DList observed,
                            traact::spatial::Pose6D cam2world,
                            vision::CameraCalibration calibration,
                            vision::Position3DList model)
        : calibration_(calibration) {
        traact::spatial::Rotation3D rot(cam2world.rotation());
        rot = rot;
        auto pos = cam2world.translation();
        camera[0] = rot.w();
        camera[1] = rot.x();
        camera[2] = rot.y();
        camera[3] = rot.z();

        camera[4] = pos.x();
        camera[5] = pos.y();
        camera[6] = pos.z();

        observed_.reserve(observed.size());
        for(const auto& point : observed){
          observed_.template emplace_back(point.x, point.y);
        }
        model_.reserve(observed.size());
        for(const auto& point : model){
            model_.template emplace_back(point.x, point.y, point.z);
        }
    }
    TargetReprojectionError(std::vector<Eigen::Vector2d> observed,
                            traact::spatial::Pose6D cam2world,
                            vision::CameraCalibration calibration,
                            std::vector<Eigen::Vector3d> model)
        : observed_(observed),model_(model), calibration_(calibration) {
        traact::spatial::Rotation3D rot(cam2world.rotation());
        rot = rot;
        auto pos = cam2world.translation();
        camera[0] = rot.w();
        camera[1] = rot.x();
        camera[2] = rot.y();
        camera[3] = rot.z();

        camera[4] = pos.x();
        camera[5] = pos.y();
        camera[6] = pos.z();
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

        T cam2world_rot[4];
        T cam2world_pos[3];
        for (int i = 0; i < 4; ++i)
            cam2world_rot[i] = T(camera[i]);
        for (int i = 0; i < 3; ++i)
            cam2world_pos[i] = T(camera[i + 4]);

        T cam2target_rot[4];
        T cam2target_pos[3];

        ceres::QuaternionProduct(cam2world_rot, world2target_rot, cam2target_rot);

        ceres::QuaternionRotatePoint(cam2world_rot, world2target_pos, cam2target_pos);
        cam2target_pos[0] += cam2world_pos[0];
        cam2target_pos[1] += cam2world_pos[1];
        cam2target_pos[2] += cam2world_pos[2];


        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.

        // Compute final projected point position.
        const T fx = T(calibration_.fx);
        const T fy = T(calibration_.fy);

        const T cx = T(calibration_.cx);
        const T cy = T(calibration_.cy);

        T cam2point[3];

        for (int i = 0; i < N; ++i) {
            T point[3];
            point[0] = T(model_[i].x());
            point[1] = T(model_[i].y());
            point[2] = T(model_[i].z());

            ceres::QuaternionRotatePoint(cam2target_rot, point, cam2point);
            cam2point[0] += cam2target_pos[0];
            cam2point[1] += cam2target_pos[1];
            cam2point[2] += cam2target_pos[2];

            const T px = cam2point[0];
            const T py = cam2point[1];
            const T pz = cam2point[2];

            const T xp = fx * px + cx * pz;
            const T yp = fy * py + cy * pz;
            //const T zp = pz;

            const T predicted_x = xp / pz;
            const T predicted_y = yp / pz;


            // The error is the difference between the predicted and observed position.
            residuals[i * 2 + 0] = predicted_x - T(observed_[i].x());
            residuals[i * 2 + 1] = predicted_y - T(observed_[i].y());
        }

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(vision::Position2DList observed,
                                       traact::spatial::Pose6D cam2world,
                                       vision::CameraCalibration calibration,
                                       vision::Position3DList model) {
        return (new ceres::AutoDiffCostFunction<
            TargetReprojectionError, N * 2, 7>(
            new TargetReprojectionError(observed, cam2world,
                                        calibration, model)));
    }


    double camera[7];
    const vision::CameraCalibration calibration_;
    std::vector<Eigen::Vector3d> model_;
    std::vector<Eigen::Vector2d> observed_;
};

class TargetReprojectionErrorFactory {
 public:
    static ceres::CostFunction *Create(vision::Position2DList observed,
                                       traact::spatial::Pose6D cam2world,
                                       vision::CameraCalibration calibration,
                                       vision::Position3DList model) {

        size_t point_count = observed.size();
        switch (point_count) {
            case 1:return TargetReprojectionError<1>::Create(observed, cam2world, calibration, model);
            case 2:return TargetReprojectionError<2>::Create(observed, cam2world, calibration, model);
            case 3:return TargetReprojectionError<3>::Create(observed, cam2world, calibration, model);
            case 4:return TargetReprojectionError<4>::Create(observed, cam2world, calibration, model);
            case 5:return TargetReprojectionError<5>::Create(observed, cam2world, calibration, model);
            case 6:return TargetReprojectionError<6>::Create(observed, cam2world, calibration, model);
            case 7:return TargetReprojectionError<7>::Create(observed, cam2world, calibration, model);
            case 8:return TargetReprojectionError<8>::Create(observed, cam2world, calibration, model);
            case 9:return TargetReprojectionError<9>::Create(observed, cam2world, calibration, model);
            case 10:return TargetReprojectionError<10>::Create(observed, cam2world, calibration, model);
            case 11:return TargetReprojectionError<11>::Create(observed, cam2world, calibration, model);
            case 12:return TargetReprojectionError<12>::Create(observed, cam2world, calibration, model);
            case 13:return TargetReprojectionError<13>::Create(observed, cam2world, calibration, model);
            case 14:return TargetReprojectionError<14>::Create(observed, cam2world, calibration, model);
            case 15:return TargetReprojectionError<15>::Create(observed, cam2world, calibration, model);
            case 16:return TargetReprojectionError<16>::Create(observed, cam2world, calibration, model);
            case 0:
            default:SPDLOG_ERROR("unsupported number of observations for CeresTargetNPointReprojectionError");
                return 0;

        }

    }

};
}

#endif //TRAACTMULTI_TARGETREPROJECTIONERROR_H
