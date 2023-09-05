/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_CERESTARGETNPOINTREPROJECTIONERROR_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_CERESTARGETNPOINTREPROJECTIONERROR_H_

#include "CostFunctionUtils.h"

namespace traact::vision::bundle_adjustment {


    template<uint16_t N>
    struct CeresTargetNPointReprojectionError {
        CeresTargetNPointReprojectionError(const std::vector<Eigen::Vector2d> observed,
                                        const std::vector<Eigen::Matrix2d> observed_cov,
                                        const vision::CameraCalibration &mIntrinsics) : observed_(observed),
                                                                                        observed_cov_(observed_cov),
                                                                                        m_intrinsics(mIntrinsics)
                , fx(mIntrinsics.fx), fy(mIntrinsics.fy)
                , cx(mIntrinsics.cx), cy(mIntrinsics.cy){

            stddev_.resize(N, std::vector<double>(2,1));

            for(int i=0;i<N;++i){
                stddev_[i][0] = sqrt(observed_cov_[i](0,0));
                stddev_[i][1] = sqrt(observed_cov_[i](1,1));
            }

        }

        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residuals) const {



            T p2d[N][2];

            for(int i=0;i<N;i++){
                const T* p = point+i*3;
                reprojectPoint(camera, p, fx,fy,cx,cy, p2d[i]);
            }


            for(int i=0;i<N;i++){
                // The error is the difference between the predicted and observed position.
                residuals[i*2+0] = (observed_[i][0] - p2d[i][0]);// / T(stddev_[i][0]);
                residuals[i*2+1] = (observed_[i][1] - p2d[i][1]);// / T(stddev_[i][1]);
            }

//            {
//                std::cout << "CeresTargetNPointReprojectionError\n";
//                for(int i=0;i<N;i++){
//                    // The error is the difference between the predicted and observed position.
//                    //std::cout << "observed_x " << observed_[i][0] << " predicted_x " << p2d[i][0]  << "\n";
//                    //std::cout << "observed_y " << observed_[i][1] << " predicted_y " << p2d[i][1]  << "\n";
//                    std::cout << "residuals " << residuals[i*2+0] << " " << residuals[i*2+1] << std::endl;
//                }
//            }

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const std::vector<Eigen::Vector2d> observed,
                                           const std::vector<Eigen::Matrix2d> observed_cov,
                                           const traact::vision::CameraCalibration intrinsics) {


            return (new ceres::AutoDiffCostFunction< CeresTargetNPointReprojectionError, N*2, 7, N*3 >(
                    new CeresTargetNPointReprojectionError<N>(observed, observed_cov,intrinsics)));
        }

        const std::vector<Eigen::Vector2d>  observed_;
        const std::vector<Eigen::Matrix2d> observed_cov_;

        std::vector<std::vector<double>> stddev_;

        const double fx;
        const double fy;

        const double cx;
        const double cy;

        traact::vision::CameraCalibration m_intrinsics;
    };

    class CeresTargetNPointReprojectionErrorFactory {
    public:
        static ceres::CostFunction* Create(const std::vector<Eigen::Vector2d>& observed,
                                           const std::vector<Eigen::Matrix2d>& observed_cov,
                                           const traact::vision::CameraCalibration& intrinsics) {

            std::size_t point_count = observed.size();
            switch (point_count) {
                case 1:
                    return CeresTargetNPointReprojectionError<1>::Create(observed, observed_cov, intrinsics);
                case 2:
                    return CeresTargetNPointReprojectionError<2>::Create(observed, observed_cov, intrinsics);
                case 3:
                    return CeresTargetNPointReprojectionError<3>::Create(observed, observed_cov, intrinsics);
                case 4:
                    return CeresTargetNPointReprojectionError<4>::Create(observed, observed_cov, intrinsics);
                case 5:
                    return CeresTargetNPointReprojectionError<5>::Create(observed, observed_cov, intrinsics);
                case 6:
                    return CeresTargetNPointReprojectionError<6>::Create(observed, observed_cov, intrinsics);
                case 7:
                    return CeresTargetNPointReprojectionError<7>::Create(observed, observed_cov, intrinsics);
                case 8:
                    return CeresTargetNPointReprojectionError<8>::Create(observed, observed_cov, intrinsics);
                case 9:
                    return CeresTargetNPointReprojectionError<9>::Create(observed, observed_cov, intrinsics);
                case 10:
                    return CeresTargetNPointReprojectionError<10>::Create(observed, observed_cov, intrinsics);
                case 11:
                    return CeresTargetNPointReprojectionError<11>::Create(observed, observed_cov, intrinsics);
                case 12:
                    return CeresTargetNPointReprojectionError<12>::Create(observed, observed_cov, intrinsics);
                case 13:
                    return CeresTargetNPointReprojectionError<13>::Create(observed, observed_cov, intrinsics);
                case 14:
                    return CeresTargetNPointReprojectionError<14>::Create(observed, observed_cov, intrinsics);
                case 15:
                    return CeresTargetNPointReprojectionError<15>::Create(observed, observed_cov, intrinsics);
                case 16:
                    return CeresTargetNPointReprojectionError<16>::Create(observed, observed_cov, intrinsics);
                case 0:
                default:
                    spdlog::error("unsupported number of observations for CeresTargetNPointReprojectionError");
                    return 0;

            }


        }
    };
}


#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_CERESTARGETNPOINTREPROJECTIONERROR_H_
