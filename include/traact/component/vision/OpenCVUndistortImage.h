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

#ifndef TRAACTMULTI_OPENCVUNDISTORTIMAGE_H
#define TRAACTMULTI_OPENCVUNDISTORTIMAGE_H


#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/UndistortionHelper.h>

namespace traact::component::vision {

    class OpenCVUndistortImage : public Component {
    public:
        explicit OpenCVUndistortImage(const std::string &name) : Component(name,
                                                                         traact::component::ComponentType::Functional) {
        }

        static traact::pattern::Pattern::Ptr getPattern() {


            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("OpenCVUndistortImage", unlimited);

            pattern->addConsumerPort("input_image", traact::vision::ImageHeader::MetaType);
            pattern->addConsumerPort("input_calibration", traact::vision::CameraCalibrationHeader::MetaType);
            pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);

            pattern->addCoordianteSystem("ImagePlane")
                    .addCoordianteSystem("Image", true)
                    .addEdge("ImagePlane", "Image", "input_image")
                    .addEdge("ImagePlane", "Image", "input_calibration")
                    .addEdge("ImagePlane", "Image", "output");

            return pattern;
        }

        bool processTimePoint(buffer::GenericComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& image = data.getInput<ImageHeader::NativeType, ImageHeader>(0);
            const auto& calibration = data.getInput<CameraCalibrationHeader::NativeType, CameraCalibrationHeader>(1);

            auto& output = data.getOutput<ImageHeader::NativeType, ImageHeader>(0);

            undistorter_.Init(calibration, false, true, 0);

            return undistorter_.UndistortImage(image, output);
        }


    private:
        ::traact::vision::UndistortionHelper undistorter_;

    };

}




#endif //TRAACTMULTI_OPENCVUNDISTORTIMAGE_H