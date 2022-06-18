/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
#define TRAACTMULTI_SUBPIXELEDGEDECTECTION_H

namespace traact::vision {
template<class T>
class SubPixelEdgeDectection {
 public:
    virtual ~SubPixelEdgeDectection() {};
    virtual traact::Scalar findEdge(T *startInImage,
                                    size_t image_size,
                                    T *image_end,
                                    const int positionIncrement,
                                    T threshold,
                                    const int maxDistance) = 0;
};

template<class T>
class ThresholdSubPixelEdgeDectection {
 public:
    ~ThresholdSubPixelEdgeDectection() {};
    traact::Scalar findEdge(T *startInImage, size_t image_size,const int positionIncrement, T threshold, const int maxDistance) {

        for (int i = 0; i < maxDistance; ++i) {
            int imageIndex = i * positionIncrement;
            if(imageIndex < 0 || imageIndex > image_size){
                return 0;
            }

            if (startInImage[imageIndex] < threshold) {
                int prev_index = imageIndex - positionIncrement;

                T prev_value = startInImage[prev_index];
                T value = startInImage[imageIndex];

                traact::Scalar delta =
                    static_cast<traact::Scalar>(prev_value - threshold) / static_cast<traact::Scalar>(prev_value - value);

                return static_cast<traact::Scalar>(i) + delta;
            }
        }

        return 0;
    }
};
}

#endif //TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
