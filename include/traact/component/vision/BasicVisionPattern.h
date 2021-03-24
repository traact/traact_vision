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

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_BASICVISIONPATTERN_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_BASICVISIONPATTERN_H_

#include <traact/pattern/spatial/SpatialPattern.h>
#include <traact/datatypes.h>
#include <traact/vision.h>
namespace traact::component::vision {
static traact::pattern::spatial::SpatialPattern::Ptr getUncalibratedCameraPattern() {

  traact::pattern::spatial::SpatialPattern::Ptr
      pattern =
      std::make_shared<traact::pattern::spatial::SpatialPattern>("UncalibratedCameraPattern", serial);

  pattern->addProducerPort("output", traact::vision::ImageHeader::MetaType);
  pattern->addCoordianteSystem("ImagePlane")
      .addCoordianteSystem("Image", true)
      .addEdge("ImagePlane", "Image", "output");

  std::set<std::string> pixel_formats;
  pixel_formats.emplace("Luminance");
  pattern->addParameter("width", 640, 1)
  .addParameter("height", 320,1)
  .addParameter("PixelFormat", "Luminance", pixel_formats);

  return pattern;
}

static traact::pattern::spatial::SpatialPattern::Ptr getCameraPattern() {

  traact::pattern::spatial::SpatialPattern::Ptr
      pattern = getUncalibratedCameraPattern();

  pattern->addProducerPort("output_calibration", traact::vision::CameraCalibrationHeader::MetaType);

  pattern->addCoordianteSystem("Camera")
      .addEdge("ImagePlane", "Camera", "intrinsic");



  return pattern;
}

}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_BASICVISIONPATTERN_H_
