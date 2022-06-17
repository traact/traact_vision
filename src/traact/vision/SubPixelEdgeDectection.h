/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
#define TRAACTMULTI_SUBPIXELEDGEDECTECTION_H

namespace traact::vision {
template<class T>
class SubPixelEdgeDectection {
 public:
    virtual ~SubPixelEdgeDectection() {};
    virtual traact::Scalar findEdge(T *startInImage, const int positionIncrement, T threshold, const int maxDistance) = 0;
};

template<class T>
class ThresholdSubPixelEdgeDectection {
 public:
    ~ThresholdSubPixelEdgeDectection() {};
    traact::Scalar findEdge(T *startInImage, const int positionIncrement, T threshold, const int maxDistance) {

        for (int i = 0; i < maxDistance; ++i) {
            int imageIndex = i * positionIncrement;

            if (startInImage[imageIndex] < threshold) {
                if (i == 0) return 0;

                int prevIndex = imageIndex - positionIncrement;

                T prevvalue = startInImage[prevIndex];
                T value = startInImage[imageIndex];

                traact::Scalar delta = static_cast<traact::Scalar>(prevvalue - threshold) / static_cast<traact::Scalar>(prevvalue - value);

                return static_cast<traact::Scalar>(i) + delta;
            }
        }

        return 0;
    }
};
}

#endif //TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
