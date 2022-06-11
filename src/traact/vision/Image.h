/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_IMAGE_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_IMAGE_H_

#include <utility>
#include <memory>
#include "traact/traact_vision_export.h"
#include "traact/datatypes.h"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

namespace traact::vision {

class TRAACT_VISION_EXPORT Image {
    public:
    Image() = default;
    Image(Image const& image) = default;
    Image& operator=(Image const& image) =default;
    Image(Image && image) = default;
    Image& operator=(Image && image) = default;

    [[nodiscard]] const cv::Mat& getImage() const;

    [[nodiscard]] cv::Mat& getImage();


    void update(cv::Mat image);

    template<typename T>
    void update(cv::Mat image, std::shared_ptr<T> && owner){
        SPDLOG_TRACE("update image content with external data owner");
        owner_ = std::move(owner);
        image_ = std::move(image);
    }

    private:
    cv::Mat image_;
    std::shared_ptr<void> owner_{};

};

} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_IMAGE_H_
