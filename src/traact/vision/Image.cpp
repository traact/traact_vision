/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "Image.h"

namespace traact::vision {

Image::Image(const Image &image) {
    value_ = image.value_;
    owner_ = image.owner_;

}
Image &Image::operator=(const Image &image) {
    value_ = image.value_;
    owner_ = image.owner_;
}
} // traact::vision