/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
#define TRAACTMULTI_SUBPIXELEDGEDECTECTION_H

namespace traact::vision {
template<class T>
class SubPixelEdgeDectection {
 public:
    virtual ~SubPixelEdgeDectection() {};
    virtual double findEdge(T *startInImage, const int positionIncrement, T threshold, const int maxDistance) = 0;
};

template<class T>
class ThresholdSubPixelEdgeDectection {
 public:
    ~ThresholdSubPixelEdgeDectection() {};
    double findEdge(T *startInImage, const int positionIncrement, T threshold, const int maxDistance) {

        for (int i = 0; i < maxDistance; ++i) {
            int imageIndex = i * positionIncrement;

            if (startInImage[imageIndex] < threshold) {
                if (i == 0) return 0;

                int prevIndex = imageIndex - positionIncrement;

                T prevvalue = startInImage[prevIndex];
                T value = startInImage[imageIndex];

                double delta = static_cast<double>(prevvalue - threshold) / static_cast<double>(prevvalue - value);

                return static_cast<double>(i) + delta;
            }
        }

        return 0;
    }
};
}

#endif //TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
