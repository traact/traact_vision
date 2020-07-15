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

#ifndef TRAACTMULTI_PERSPECTIVE_H
#define TRAACTMULTI_PERSPECTIVE_H

#include <traact/vision_datatypes.h>
#include <traact/datatypes.h>

#include <traact/math/utils.h>

namespace traact::math {
    Eigen::Vector2d reproject_point(const vision::CameraCalibration& intrinsics, const Eigen::Vector3d& point);
    Eigen::Vector2d reproject_point(const Eigen::Affine3d& cam2world, const traact::vision::CameraCalibration &intrinsics, const Eigen::Vector3d& point);

    bool estimate_camera_pose(Eigen::Affine3d& pose_result, const std::vector<Eigen::Vector2d>& image_points, const vision::CameraCalibration& intrinsics, const std::vector<Eigen::Vector3d>& model_points);

    bool estimate_3d_point(Eigen::Vector3d& result,  const std::vector<Eigen::Affine3d>& world2camera, const std::vector<vision::CameraCalibration>& intrinsics, const std::vector<Eigen::Vector2d> image_point);

    Eigen::Matrix<double, 3, 4> create_projection_matrix(const Eigen::Affine3d& cam2world, const vision::CameraCalibration calibration);
}

#endif //TRAACTMULTI_PERSPECTIVE_H
