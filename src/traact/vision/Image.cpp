/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "Image.h"

namespace traact::vision {
const cv::Mat& Image::getImage() const{
    return image_;
}
cv::Mat &Image::getImage() {
    return image_;
}
void Image::update(cv::Mat image) {
    SPDLOG_TRACE("update image content with internal data owner");
    owner_.reset();
    image_ = image;

}

} // traact::vision