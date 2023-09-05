/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_COSTFUNCTIONUTILS_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_COSTFUNCTIONUTILS_H_

#include <traact/vision.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <sstream>

namespace traact::vision::bundle_adjustment {
    template<typename T> inline void reprojectPoint(const T* const camera,const T* const point, const double fx, const double fy, const double cx, const double cy, T* reprojectedPoint) {

        // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
        //
        // We use QuaternionRotatePoint as it does not assume that the
        // quaternion is normalized, since one of the ways to run the
        // bundle adjuster is to let Ceres optimize all 4 quaternion
        // parameters without a local parameterization.
        T p[3];
        ceres::QuaternionRotatePoint(camera, point, p);

        p[0] += camera[4];
        p[1] += camera[5];
        p[2] += camera[6];


        const T xp = fx * p[0] + cx * p[2];
        const T yp = fy * p[1] + cy * p[2];
        const T zp = p[2];

        reprojectedPoint[0] = xp / zp;
        reprojectedPoint[1] = yp / zp;

    };

    constexpr std::size_t factorial(std::size_t n)
    {
        return n <= 1 ? 1 : (n * factorial(n - 1));
    }

    constexpr std::size_t count3DDistances(std::size_t n){
        return n <= 1 ? 0 : (n-1 + count3DDistances(n - 1));
    }

    template<typename T>
    void calculate3DDistancesSquared(const T* const point_data, T* result, std::size_t N) {

        if(N < 2)
            return;

        T x0 = point_data[0];
        T y0 = point_data[1];
        T z0 = point_data[2];
        for(std::size_t i=1;i<N;++i) {
            T xn = point_data[i*3+0];
            T yn = point_data[i*3+1];
            T zn = point_data[i*3+2];

            T xd = x0 - xn;
            T yd = y0 - yn;
            T zd = z0 - zn;

            T d = xd*xd + yd*yd + zd*zd;

            result[i-1] = d;
        }

        calculate3DDistancesSquared<T>(point_data + 3, result + N - 1, N - 1);

    }


}

#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_COSTFUNCTIONUTILS_H_
