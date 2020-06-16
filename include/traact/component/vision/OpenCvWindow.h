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

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVWINDOW_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVWINDOW_H_

#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/highgui.hpp>

namespace traact::component::vision {

class OpenCvWindow : public traact::DefaultComponent {
 public:
  explicit OpenCvWindow(const std::string &name) : traact::DefaultComponent(name,
                                                                            traact::component::ComponentType::SyncSink) {
  }

  static traact::pattern::Pattern::Ptr getPattern() {
    using namespace traact::vision;
    traact::pattern::spatial::SpatialPattern::Ptr
        pattern =
        std::make_shared<traact::pattern::spatial::SpatialPattern>("OpenCvWindow", serial);

    pattern->addConsumerPort("input", ImageHeader::MetaType);

    return pattern;
  }

  bool init() override {

    cv::namedWindow( GetWindowName(), cv::WINDOW_AUTOSIZE );
    cv::startWindowThread();
    return Component::init();
  }
  bool teardown() override {
    return Component::teardown();
  }

  bool processTimePoint(traact::DefaultComponentBuffer &data) override {
    using namespace traact::vision;
    const auto &input = data.getInput<ImageHeader::NativeType, ImageHeader>(0);

    auto image = input.GetCpuMat();
    cv::imshow( GetWindowName(), image );
    cv::waitKey(1);


    return true;

  }

  std::string GetWindowName() {
    return getName();
  }

};

}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVWINDOW_H_
