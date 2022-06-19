/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_CERESREFNPOINTREPROJECTIONERROR_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_CERESREFNPOINTREPROJECTIONERROR_H_

#include "CostFunctionUtils.h"

namespace traact::vision::bundle_adjustment {


    template<uint16_t N>
    struct CeresRefNPointReprojectionError {
        CeresRefNPointReprojectionError(const std::vector<Eigen::Vector2d> observed,
                                        const std::vector<Eigen::Matrix2d> observed_cov,
                                        const std::vector<Eigen::Vector3d> ref_point,
                                        const vision::CameraCalibration &mIntrinsics) : observed_(observed),
                                                                                        observed_cov_(observed_cov),
                                                                                        ref_points_(ref_point),
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
                        T* residuals) const {



            T p3d[N][3];


            for(int i=0;i<N;i++){
                for(int j=0;j<3;j++){
                    p3d[i][j] = T(ref_points_[i][j]);
                }
            }


            T p[N][2];
            for(int i=0;i<N;i++){
                reprojectPoint(camera, p3d[i], fx,fy,cx,cy,p[i]);
            }

            for(int i=0;i<N;i++){
                // The error is the difference between the predicted and observed position.
                residuals[i*2+0] = (observed_[i][0] - p[i][0]) / T(stddev_[i][0]);
                residuals[i*2+1] = (observed_[i][1] - p[i][1]) / T(stddev_[i][1]);
            }


//            {
//                std::stringstream  ss;
//                ss << "UbitrackWandReprojectionError "  << "\n";
//                //ss << "observed_x " << observed_1_[0] << " predicted_x " << p1[0]  << "\n";
//                //ss<< "observed_y " << observed_1_[1] << " predicted_y " << p1[1] << std::endl;
//                ss << "residuals p1 " << residuals[0] << " " << residuals[1] << std::endl;
//                ss << "residuals p2 " << residuals[2] << " " << residuals[3] << std::endl;
//                ss << "residuals p3 " << residuals[4] << " " << residuals[5] << std::endl;
//                ss << "residuals p4 " << residuals[6] << " " << residuals[7] << std::endl;
//
//                ss <<  "\n";
//                spdlog::info(ss.str());
//            }



            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const std::vector<Eigen::Vector2d> observed,
                                           const std::vector<Eigen::Matrix2d> observed_cov,
                                           const std::vector<Eigen::Vector3d> ref_point,
                                           const traact::vision::CameraCalibration intrinsics) {


            return (new ceres::AutoDiffCostFunction< CeresRefNPointReprojectionError, N*2, 7 >(
                    new CeresRefNPointReprojectionError<N>(observed, observed_cov,
                                                        ref_point,intrinsics)));
        }

        const std::vector<Eigen::Vector2d> observed_;
        const std::vector<Eigen::Matrix2d> observed_cov_;

        std::vector<std::vector<double>> stddev_;

        const std::vector<Eigen::Vector3d> ref_points_;

        const double fx;
        const double fy;

        const double cx;
        const double cy;

        traact::vision::CameraCalibration m_intrinsics;
    };
}


#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_COST_FUNCTION_CERESREFNPOINTREPROJECTIONERROR_H_
