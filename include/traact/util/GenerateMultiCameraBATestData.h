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

#ifndef TRAACTMULTI_GENERATEMULTICAMERABATESTDATA_H
#define TRAACTMULTI_GENERATEMULTICAMERABATESTDATA_H

#include <traact/spatial.h>
#include <traact/vision.h>

namespace traact::util {

    struct GenerateMultiCameraBATestDataCamera {
        spatial::Pose6D camera2world;
        spatial::Pose6D camera2world_noise;
        vision::CameraCalibration calibration;

    };
    class GenerateMultiCameraBATestData {
    public:
        void Init(std::size_t data_count, std::size_t camera_count, std::size_t width, std::size_t height,
                  spatial::Position3DList model_points, double point2d_noise, double camera_pos_noise,
                  double camera_rot_noise,
                  std::size_t noise_point_count);
        std::vector<GenerateMultiCameraBATestDataCamera> GetCameraData();


        spatial::Position2DList GetPointsForCamera(std::size_t idx);
        bool Next();

        spatial::Pose6D GetTargetPose(std::size_t data_idx);
        spatial::Position2DList GetPointsForCamera(std::size_t data_idx,std::size_t camera_idx);
        spatial::Position2DList GetPointsForCameraNoise(std::size_t data_idx,std::size_t camera_idx);

    protected:
        std::size_t max_count_;
        std::size_t current_count_{0};
        double point2d_noise_{0};
        double camera_pos_noise_{0};
        double camera_rot_noise_{0};
        std::vector<GenerateMultiCameraBATestDataCamera> camera_data_;
        std::vector<spatial::Pose6D> expected_pose_;
        std::vector<std::vector<spatial::Position2DList>> reference_point2d_;
        std::vector<std::vector<spatial::Position2DList>> reference_point2d_noise_;
        spatial::Position3DList model_points_;

    };
}




#endif //TRAACTMULTI_GENERATEMULTICAMERABATESTDATA_H
