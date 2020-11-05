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

#ifndef TRAACTMULTI_UNDISTORTIONHELPER_H
#define TRAACTMULTI_UNDISTORTIONHELPER_H

#include <traact/vision_datatypes.h>
#include <mutex>

namespace traact::vision {

    class UndistortionHelper {
    public:
        UndistortionHelper() = default;
        UndistortionHelper(const UndistortionHelper& other);
        ~UndistortionHelper() = default;


        void Init(const CameraCalibration& calibration, bool optimize_intrinsics = false, bool center_principle_point = true, double alpha = 0);

        CameraCalibration GetUndistortedCalibration();

        bool UndistortImage(const Image& input, Image& output  );
        bool UndistortImage(const cv::Mat& input, cv::Mat& output  );

    protected:
        cv::Mat mapX_;
        cv::Mat mapY_;
        CameraCalibration distorted_calibration_;
        CameraCalibration undistorted_calibration_;
        std::mutex init_mutex_;


    };


}



#endif //TRAACTMULTI_UNDISTORTIONHELPER_H
