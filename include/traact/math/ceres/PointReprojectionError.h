/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef TRAACTMULTI_POINTREPROJECTIONERROR_H
#define TRAACTMULTI_POINTREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <traact/vision_datatypes.h>

namespace traact::math {


    struct PointReprojectionError {
        // (u, v): the position of the observation with respect to the image
        // center point.
        PointReprojectionError(Eigen::Vector2d observed, Eigen::Affine3d cam2world, vision::CameraCalibration calibration, int index)
                : observed_(observed), calibration_(calibration), observationIndex_(index)
        {
            Eigen::Quaterniond rot(cam2world.rotation());
            rot = rot ;
            auto pos = cam2world.translation();
            camera[0] = rot.w();
            camera[1] = rot.x();
            camera[2] = rot.y();
            camera[3] = rot.z();

            camera[4] = pos.x();
            camera[5] = pos.y();
            camera[6] = pos.z();
        }

        template <typename T>
        bool operator()(const T* const point,
                        T* residuals) const {
            // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
            //
            // We use QuaternionRotatePoint as it does not assume that the
            // quaternion is normalized, since one of the ways to run the
            // bundle adjuster is to let Ceres optimize all 4 quaternion
            // parameters without a local parameterization.

            T p[3];
            T cam_tmp[7];
            for(int i=0;i<7;++i)
                cam_tmp[i] = T(camera[i]);

            ceres::QuaternionRotatePoint(cam_tmp, point, p);

            p[0] += camera[4];
            p[1] += camera[5];
            p[2] += camera[6];



            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.

            // Compute final projected point position.
            const T fx = T(calibration_.fx);
            const T fy = T(calibration_.fy);

            const T cx = T(calibration_.cx);
            const T cy = T(calibration_.cy);

            const T xp = fx * p[0] + cx * p[2];
            const T yp = fy * p[1] + cy * p[2];
            const T zp = p[2];

            const T predicted_x = xp / zp;
            const T predicted_y = yp / zp;


            // The error is the difference between the predicted and observed position.
            residuals[0] = predicted_x - T(observed_.x());
            residuals[1] = predicted_y - T(observed_.y());



            if(observationIndex_ < 0) {
                std::cout << "observationIndex " << observationIndex_  << "\n";
                std::cout << "observed_x " << observed_.x() << " predicted_x " << predicted_x  << "\n";
                std::cout << "observed_y " << observed_.y() << " predicted_y " << predicted_y << std::endl;
                std::cout << "residuals " << residuals[0] << " " << residuals[1] << std::endl;
            }





            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(Eigen::Vector2d observed, Eigen::Affine3d cam2world, vision::CameraCalibration calibration, int index) {
            return (new ceres::AutoDiffCostFunction<
                    PointReprojectionError, 2, 3>(
                    new PointReprojectionError(observed,cam2world,
                                               calibration, index)));
        }

        const Eigen::Vector2d observed_;
        const int observationIndex_;
        double camera[7];
        const vision::CameraCalibration calibration_;

    };
}

#endif //TRAACTMULTI_POINTREPROJECTIONERROR_H
