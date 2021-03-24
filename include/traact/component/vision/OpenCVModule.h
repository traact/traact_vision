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

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVMODULE_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVMODULE_H_

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/spatial.h>
#include <opencv2/highgui.hpp>
#include <map>
#include <queue>
#include <traact/util/Semaphore.h>

namespace traact::component::vision {
class OpenCVModule : public Module {
 public:
  OpenCVModule();
  bool init(ComponentPtr module_component) override;
  bool start(ComponentPtr module_component) override;
  bool stop(ComponentPtr module_component) override;
  bool teardown(ComponentPtr module_component) override;

  void addWindow(std::string window_name);
  void updateWindow(const std::string &window_name, std::size_t mea_idx,
                    buffer::BorrowedBuffer<::traact::vision::ImageHeader::NativeType>::Ptr image);
  void updateWindow(const std::string &window_name,
                    buffer::BorrowedBuffer<::traact::spatial::Position2DListHeader::NativeType>::Ptr image,
                    std::size_t mea_idx);

  //void updateWindow(const std::string& window_name, const cv::Mat& image);

 private:
  std::shared_ptr<std::thread> thread_;
  bool running_{false};
  void thread_loop();
  //std::map<std::string, buffer::BorrowedBuffer<::traact::vision::ImageHeader::NativeType>::Ptr > images_;
  std::map<std::string,  std::map<std::size_t, buffer::BorrowedBuffer<::traact::vision::ImageHeader::NativeType>::Ptr > > images_;
  std::map<std::string,  std::map<std::size_t, buffer::BorrowedBuffer<::traact::spatial::Position2DListHeader::NativeType>::Ptr > > points_;
  //std::map<std::string, cv::Mat > images_;
  std::list<std::string> windows_;
    std::map<std::string, bool> has_window_;
  std::map<std::string, std::size_t> draw_count_;
  tbb::queuing_mutex data_lock_;
    tbb::queuing_mutex running_lock_;
  WaitForInit init_lock_;
  cv::Mat invalid_image_;
  RTTR_ENABLE(Module)
};

class OpenCVComponent : public ModuleComponent {
 public:
  OpenCVComponent(const std::string &name);
  std::string GetModuleKey() override;
  Module::Ptr InstantiateModule() override;
  bool configure(const nlohmann::json &parameter, buffer::GenericComponentBufferConfig *data) override;
 protected:
  std::shared_ptr<OpenCVModule> opencv_module_;
  RTTR_ENABLE(ModuleComponent)
};

}


#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_OPENCVMODULE_H_
