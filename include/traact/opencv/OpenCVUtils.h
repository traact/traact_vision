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

#ifndef TRAACTMULTI_OPENCVUTILS_H
#define TRAACTMULTI_OPENCVUTILS_H

#include <traact/vision_datatypes.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace traact{

    static inline cv::Point2d eigen2cv(const Eigen::Vector2d p){
        return cv::Point2d (p.x(),p.y());
    }

    static inline cv::Point3d eigen2cv(const Eigen::Vector3d p){
        return cv::Point3d (p.x(),p.y(),p.z());
    }


    static inline void traact2cv(const vision::CameraCalibration& calibration, cv::Mat& intrinsics, cv::Mat& distortion) {
        cv::Mat opencv_intrinsics(3, 3, cv::DataType<double>::type);
        opencv_intrinsics.at<double>(0,0) = calibration.fx;
        opencv_intrinsics.at<double>(1,0) = 0;
        opencv_intrinsics.at<double>(2,0) = 0;

        opencv_intrinsics.at<double>(0,1) = 0;
        opencv_intrinsics.at<double>(1,1) = calibration.fy;
        opencv_intrinsics.at<double>(2,1) = 0;

        opencv_intrinsics.at<double>(0,2) = calibration.cx;
        opencv_intrinsics.at<double>(1,2) = calibration.cy;
        opencv_intrinsics.at<double>(2,2) = 1;

        intrinsics = opencv_intrinsics;

        size_t count_parameter = calibration.radial_distortion.size() + calibration.tangential_distortion.size();
        cv::Mat opencv_distortion;
        if(count_parameter < 4) {
            opencv_distortion = cv::Mat(4,1,cv::DataType<double>::type);
            opencv_distortion.at<double>(0) = 0;
            opencv_distortion.at<double>(1) = 0;
            opencv_distortion.at<double>(2) = 0;
            opencv_distortion.at<double>(3) = 0;

        } else {
            opencv_distortion = cv::Mat(count_parameter,1,cv::DataType<double>::type);

            int parameter_index = 0;
            for(int i=0;i<2;++i) {
                opencv_distortion.at<double>(parameter_index) = calibration.radial_distortion[i];
                ++parameter_index;
            }

            for(int i=0;i<calibration.tangential_distortion.size();++i) {
                opencv_distortion.at<double>(parameter_index) = calibration.tangential_distortion[i];
                ++parameter_index;
            }

            for(int i=parameter_index;i<calibration.radial_distortion.size();++i) {
                opencv_distortion.at<double>(parameter_index) = calibration.radial_distortion[i];
                ++parameter_index;
            }

        }

        distortion = opencv_distortion;

    }

    static inline void cv2traact(vision::CameraCalibration& calibration, const cv::Mat& intrinsics, const cv::Mat& distortion, const cv::Size& size) {
        calibration.fx = intrinsics.at<double>(0,0);
        calibration.fy = intrinsics.at<double>(1,1);

        calibration.cx = intrinsics.at<double>(0,2);
        calibration.cy = intrinsics.at<double>(1,2);
        calibration.skew = 0;
        calibration.radial_distortion.clear();
        calibration.tangential_distortion.clear();
        calibration.width = size.width;
        calibration.height = size.height;
        if(distortion.rows >= 4) {
            calibration.radial_distortion.push_back(distortion.at<double>(0));
            calibration.radial_distortion.push_back(distortion.at<double>(1));

            calibration.tangential_distortion.push_back(distortion.at<double>(2));
            calibration.tangential_distortion.push_back(distortion.at<double>(3));
        }

        if(distortion.rows > 4) {
            for(int i=4; i< distortion.rows;++i) {
                calibration.radial_distortion.push_back(distortion.at<double>(i));
            }
        }


    }
}

#endif //TRAACTMULTI_OPENCVUTILS_H
