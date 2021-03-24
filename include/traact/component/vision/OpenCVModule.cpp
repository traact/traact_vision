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

#include "OpenCVModule.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
bool traact::component::vision::OpenCVModule::init(traact::component::Module::ComponentPtr module_component) {
  return Module::init(module_component);
}
bool traact::component::vision::OpenCVModule::start(traact::component::Module::ComponentPtr module_component) {
    tbb::queuing_mutex::scoped_lock lock(running_lock_);
  if(running_)
    return true;
  SPDLOG_DEBUG("Starting OpenCV Module");
  running_ = true;
  thread_ = std::make_shared<std::thread>([this]{
    thread_loop();
  });
  //init_lock_.Wait();

  return Module::start(module_component);
}
bool traact::component::vision::OpenCVModule::stop(traact::component::Module::ComponentPtr module_component) {
    tbb::queuing_mutex::scoped_lock lock(running_lock_);
  if(!running_)
    return true;
  running_ = false;
  thread_->join();
  return Module::stop(module_component);
}
bool traact::component::vision::OpenCVModule::teardown(traact::component::Module::ComponentPtr module_component) {
  return Module::teardown(module_component);
}
void traact::component::vision::OpenCVModule::updateWindow(const std::string &window_name, std::size_t mea_idx,
                                                           buffer::BorrowedBuffer<::traact::vision::ImageHeader::NativeType>::Ptr image) {
//void traact::component::vision::OpenCVModule::updateWindow(const std::string& window_name, const cv::Mat& image){
    tbb::queuing_mutex::scoped_lock lock(data_lock_);
  //auto find_result = images_.find(window_name);

    int old_count = draw_count_[window_name];
    //SPDLOG_INFO("draw count for window {0} {1}", window_name, old_count);

    draw_count_[window_name] = 0;
    images_[window_name].emplace(mea_idx, image);
//  if(find_result != images_.end()) {
//    find_result->second = image;
//  } else {
//    images_.emplace(window_name, image);
//  }

}
void traact::component::vision::OpenCVModule::thread_loop() {

    invalid_image_ = cv::Mat(640,480, CV_8UC3, cv::Scalar(0, 0, 0));
//    for(const auto& name : windows_){
//        cv::namedWindow(name, cv::WINDOW_KEEPRATIO);
//        cv::imshow(name, invalid_image_);
//    }
//    cv::waitKey(10);
//    SPDLOG_INFO("windows visible");
//    init_lock_.SetInit(true);

  while (running_) {


    {
        tbb::queuing_mutex::scoped_lock lock(data_lock_);
        for(auto& name_image : images_) {

            auto find_window = has_window_.find(name_image.first);
            if(find_window == has_window_.end()){
                cv::namedWindow(name_image.first, cv::WINDOW_KEEPRATIO);
                has_window_.emplace(name_image.first, true);
            }

            if(!name_image.second.empty()){
                auto idx_image = *name_image.second.begin();
                auto& image = idx_image.second;
                std::size_t mea_idx = idx_image.first;
                name_image.second.erase(mea_idx);
                if(!image)
                    continue;
                if(points_[name_image.first].empty()){
                        cv::imshow(name_image.first, image->GetBuffer()->GetCpuMat());
                } else {
                    cv::Mat image_copy;
                    image->GetBuffer()->GetCpuMat().convertTo(image_copy, CV_8UC3);

                    auto point_list = points_[name_image.first].find(mea_idx);
                    if(point_list != points_[name_image.first].end()){
                        for(const auto& point : *point_list->second->GetBuffer()){
                            cv::circle(image_copy, cv::Point2d(point.x(),point.y()), 4, cv::Scalar_(255,0,0), 1);
                        }
                        points_[name_image.first].erase(point_list);
                    }



                    cv::imshow(name_image.first, image_copy);

                }

                //else
                //    cv::imshow(name_image.first, invalid_image_);
            }else {
                //cv::imshow(name_image.first, invalid_image_);
            }

          draw_count_[name_image.first]++;
         // cv::imshow(name_image.first, name_image.second);
      }
      //windows_.clear();
    }

    cv::waitKey(10);


  }
}
traact::component::vision::OpenCVModule::OpenCVModule() {}

void traact::component::vision::OpenCVModule::addWindow(std::string window_name) {
    tbb::queuing_mutex::scoped_lock lock(data_lock_);
    windows_.emplace_back(std::move(window_name));
}

void traact::component::vision::OpenCVModule::updateWindow(const std::string &window_name,
                                                           buffer::BorrowedBuffer<::traact::spatial::Position2DListHeader::NativeType>::Ptr image,
                                                           std::size_t mea_idx) {
    tbb::queuing_mutex::scoped_lock lock(data_lock_);

    points_[window_name].emplace(mea_idx, image);
}

traact::component::vision::OpenCVComponent::OpenCVComponent(const std::string &name)
    : ModuleComponent(name,ComponentType::SyncSink, ModuleType::Global) {}
std::string traact::component::vision::OpenCVComponent::GetModuleKey() {
  return "GlobalOpenCVModule";
}
traact::component::Module::Ptr traact::component::vision::OpenCVComponent::InstantiateModule() {
  return std::make_shared<OpenCVModule>();
}
bool traact::component::vision::OpenCVComponent::configure(const nlohmann::json &parameter, buffer::GenericComponentBufferConfig *data) {
  opencv_module_ = std::dynamic_pointer_cast<OpenCVModule>(module_);
  return ModuleComponent::configure(parameter, data);
}
