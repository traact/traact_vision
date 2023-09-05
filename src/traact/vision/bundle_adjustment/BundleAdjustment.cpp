/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "traact/vision/bundle_adjustment/BundleAdjustment.h"
#include "traact/vision/bundle_adjustment/cost_function/CeresTargetNPointReprojectionError.h"
#include "traact/vision/bundle_adjustment/cost_function/CeresRefTargetNPointLengthError.h"


namespace traact::vision::bundle_adjustment {
void BundleAdjustment::AddCamera(BACamera::Ptr camera) {
    cameras_.push_back(camera);
}

void BundleAdjustment::SetTarget(BATarget::Ptr target) {
    target_ = target;
}

bool BundleAdjustment::CheckData() {
    spdlog::info("Check data consistency");
    used_ts_.clear();

    std::set<Timestamp> all_ts_set;
    for(auto cam : cameras_) {
        auto ts = cam->GetAllTimestamps();
        std::copy(ts.begin(),ts.end(),
                  std::inserter(all_ts_set, all_ts_set.end()));

    }
    std::vector<Timestamp> all_ts = std::vector<Timestamp>(all_ts_set.begin(), all_ts_set.end());
    std::sort(all_ts.begin(), all_ts.end());

    for(Timestamp ts : all_ts) {
        bool valid_ts = true;
        std::size_t cam_count = GetCameraMeaCount(ts);

        // for now at least two cameras must see the points
        // later ease this restriction for reference points (static 3d points)
        if(cam_count < 2)
            continue;
        if(!target_->HasMeasurement(ts)){
            spdlog::info("missing 3d points for target for ts {0}", ts.time_since_epoch().count());
            valid_ts = false;//TryEstimatePoints(ts);
        } else {
            //spdlog::info("present 3d points for target for ts {0}", ts.time_since_epoch().count());
        }

        if(valid_ts){
            used_ts_.push_back(ts);
        }
    }


    spdlog::info("Count of valid timestamps for bundle adjustment: {0}", used_ts_.size());

    return true;
}

bool BundleAdjustment::Optimize() {

    ceres_problem_ = std::make_shared<ceres::Problem>();

    target_points_count_ = target_->GetTargetData().size();
    target_parameter_size_ = target_points_count_*3;


    ceres_parameter_ = new double[cameras_.size() * camera_parameter_size_ + used_ts_.size() * target_parameter_size_];
    //ceres_point_parameter_ = new double[used_ts_.size() * size_point_parameter];


    for(int i=0;i<cameras_.size();++i){
        auto cam = cameras_[i];
        double *cam_parameter = GetCameraParameter(i);

        auto camera2world = cam->getExtrinsic();

        Eigen::Quaternion<Scalar> rot(camera2world.rotation());
        Eigen::Vector3<Scalar> pos(camera2world.translation());


        cam_parameter[0] = rot.w();
        cam_parameter[1] = rot.x();
        cam_parameter[2] = rot.y();
        cam_parameter[3] = rot.z();

        cam_parameter[4] = pos.x();
        cam_parameter[5] = pos.y();
        cam_parameter[6] = pos.z();

    }


    std::vector<std::size_t> camera_mea_count(cameras_.size(), 0);

    for(std::size_t i=0;i<used_ts_.size();++i){
        Timestamp ts = used_ts_[i];

        auto points3d = target_->GetMeasurement(ts);
        double* target_parameter = GetTargetParameter(i);
        for(int j=0;j<points3d.size();++j){
            target_parameter[j * 3 + 0] = points3d[j].x();
            target_parameter[j * 3 + 1] = points3d[j].y();
            target_parameter[j * 3 + 2] = points3d[j].z();
        }

        for(int cam_idx=0;cam_idx<cameras_.size();++cam_idx){
            auto cam = cameras_[cam_idx];
            if(!cam->HasMeasurement(ts))
                continue;
            camera_mea_count[cam_idx]++;
            auto points2d = cam->GetMeasurement(ts);
            auto intrinsic = cam->getIntrinsic();

            std::vector<Eigen::Matrix2d> point_cov;
            point_cov.resize(target_points_count_, Eigen::Matrix2d::Identity());

            ceres::CostFunction* cost_function = CeresTargetNPointReprojectionErrorFactory::Create(points2d, point_cov, intrinsic);

            double *cam_parameter = GetCameraParameter(cam_idx);
            ceres_problem_->AddResidualBlock(cost_function,
                                             NULL, //new ceres::HuberLoss(2), //NULL /* squared loss */,
                                             cam_parameter, target_parameter);

            if(cam->isStaticPosition() || cam->isStaticRotation()) {
                ceres_problem_->SetParameterBlockConstant(cam_parameter);
            }
        }

        if(target_->IsUseTargetResidual()) {

            ceres::CostFunction* cost_function = CeresRefTargetNPointLengthErrorFactory::Create(target_->GetTargetData(), target_->GetStdDev());

            ceres_problem_->AddResidualBlock(cost_function,
                                             NULL, //new ceres::HuberLoss(2), //NULL /* squared loss */,
                                             target_parameter);
        }
    }



    //ba::PoseLogger pose_logger(directory, &ba_world);
    ceres::Solver::Options options;
    //options.callbacks.push_back(&pose_logger);
    options.update_state_every_iteration = true;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.use_inner_iterations = true;
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;



    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-12;
    options.parameter_tolerance =1e-8;
    options.max_num_iterations = 50;

    options.num_threads = 8;


    ceres::Solver::Summary summary;

    ceres::Solve(options, ceres_problem_.get(), &summary);
    spdlog::info("{0}", summary.FullReport());

    spdlog::info("Measurements per camera:");
    for(int i=0;i<camera_mea_count.size();++i){
        spdlog::info("camera idx {0}: {1}", i, camera_mea_count[i]);
    }

//    std::ofstream report_stream((experimentdir/"final_solver_report").string());
//    report_stream << summary.FullReport();
//    report_stream.close();


    return summary.IsSolutionUsable();
}

std::vector<spatial::Pose6D> BundleAdjustment::getResult() {
    std::vector<spatial::Pose6D> result;
    for(int i=0;i<cameras_.size();++i){
        auto cam = cameras_[i];
        double* ceres_pose = GetCameraParameter(i);
        Eigen::Quaterniond rot(ceres_pose[0],ceres_pose[1],ceres_pose[2],ceres_pose[3]);
        rot.normalize();
        Eigen::Vector3d pos(ceres_pose[4],ceres_pose[5],ceres_pose[6]);


        Eigen::Affine3d camera_to_world = Eigen::Translation3d(pos) * rot;

        result.emplace_back(camera_to_world);

    }

    return result;

}

std::size_t BundleAdjustment::GetCameraMeaCount(traact::Timestamp ts) {
    std::size_t result = 0;
    for(auto cam : cameras_) {
        if(cam->HasMeasurement(ts))
            result++;
    }
    return result;
}

BundleAdjustment::~BundleAdjustment() {
    if(ceres_parameter_)
        delete ceres_parameter_;
}

double *BundleAdjustment::GetCameraParameter(std::size_t idx) {
    return ceres_parameter_ + idx * camera_parameter_size_;
}

double *BundleAdjustment::GetTargetParameter(std::size_t idx) {
    double* ceres_point_parameter_ = ceres_parameter_ + cameras_.size() * camera_parameter_size_;
    return ceres_point_parameter_ + idx * target_parameter_size_;
}




} // traact