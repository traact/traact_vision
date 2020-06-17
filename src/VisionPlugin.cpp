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

#include <rttr/registration>
#include <rttr/type>
#include <traact/facade/Plugin.h>

#include "traact/vision.h"
#include "traact/component/vision/OpenCvVideoCapture.h"
#include "traact/component/vision/OpenCvWindow.h"

namespace traact::vision {


class VisionPlugin : public traact::facade::Plugin {
 public:
  void fillDatatypeNames(std::vector<std::string> &datatype_names) override {
    datatype_names.emplace_back(ImageHeader::MetaType);
  }
  buffer::GenericFactoryObject::Ptr instantiateDataType(const std::string &datatype_name) override {
    if (datatype_name == std::string(ImageHeader::MetaType))
      return std::make_shared<ImageFactoryObject>();
    return nullptr;
  }


  void fillPatternNames(std::vector<std::string> &pattern_names) override {
    pattern_names.emplace_back("OpenCvVideoCapture");
    pattern_names.emplace_back("OpenCvWindow");
  }
  pattern::Pattern::Ptr instantiatePattern(const std::string &pattern_name) override {
    if (pattern_name == "OpenCvVideoCapture")
      return component::vision::OpenCVVideoCapture::getPattern();
    if (pattern_name == "OpenCvWindow")
      return component::vision::OpenCvWindow::getPattern();


    return nullptr;
  }
  component::Component::Ptr instantiateComponent(const std::string &pattern_name,
                                                 const std::string &new_component_name) override {

    if (pattern_name == "OpenCvVideoCapture")
      return std::make_shared<component::vision::OpenCVVideoCapture>(new_component_name);
    if (pattern_name == "OpenCvWindow")
      return std::make_shared<component::vision::OpenCvWindow>(new_component_name);


    return nullptr;
  }

  RTTR_ENABLE(traact::facade::Plugin)
};

}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

  using namespace rttr;
  registration::class_<traact::vision::VisionPlugin>("VisionPlugin").constructor<>()
      (
          //policy::ctor::as_std_shared_ptr
      );
}