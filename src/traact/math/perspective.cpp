/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "perspective.h"
#include <opencv2/core/eigen.hpp>
#include "traact/opencv/OpenCVUtils.h"
#include <traact/util/Logging.h>
#include <opencv2/calib3d.hpp>

#include <ceres/ceres.h>
#include "traact/math/ceres/PointReprojectionError.h"
#include "traact/math/ceres/TargetReprojectionError.h"
#include "traact/math/ceres/DistanceError3D3D.h"

traact::vision::Position2D
traact::math::reproject_point(const traact::vision::CameraCalibration &intrinsics, const vision::Position3D &point) {
    cv::Mat cv_intrinsics;
    cv::Mat cv_distortion;
    traact2cv(intrinsics, cv_intrinsics, cv_distortion);
    std::vector<cv::Point2f> image_points(1);
    std::vector<cv::Point3f> obj_point{point};

    cv::Mat tmp(1, 3, CV_32FC1);
    tmp.setTo(0);
    cv::projectPoints(obj_point, tmp, tmp, cv_intrinsics, cv_distortion, image_points);
    return traact::vision::Position2D(image_points[0].x, image_points[0].y);
}

traact::vision::Position2D
traact::math::reproject_point(const traact::spatial::Pose6D &cam2world,
                              const traact::vision::CameraCalibration &intrinsics,
                              const vision::Position3D &point) {

    auto p_tmp = cam2world * point;
    return reproject_point(intrinsics, p_tmp);
}

bool traact::math::estimate_camera_pose(spatial::Pose6D &pose_result, const vision::Position2DList &image_points,
                                        const traact::vision::CameraCalibration &intrinsics,
                                        const vision::Position3DList &model_points) {

    if (image_points.size() != model_points.size()) {
        SPDLOG_ERROR("size of image and model points differ");
        return false;
    }

    if (image_points.size() < 4) {
        SPDLOG_ERROR("at least four points needed, 6 if non planar");
        return false;
    }

    cv::Mat opencv_intrinsics;
    cv::Mat opencv_distortion;
    cv::Mat t_vec;
    cv::Mat r_vec;


    traact2cv(intrinsics, opencv_intrinsics, opencv_distortion);


    bool result = cv::solvePnP(model_points,
                               image_points,
                               opencv_intrinsics,
                               opencv_distortion,
                               r_vec,
                               t_vec,
                               false,
                               cv::SOLVEPNP_ITERATIVE);

    if (!result)
        return false;


    cv2traact(r_vec, t_vec, pose_result);


    return true;

}

bool traact::math::estimate_3d_point(vision::Position3D &result,
                                     const std::vector<traact::spatial::Pose6D> &cam2world,
                                     const std::vector<vision::CameraCalibration> &intrinsics,
                                     const vision::Position2DList &image_point,
                                     double *covariance_output) {
    if (intrinsics.size() != image_point.size() || cam2world.size() != image_point.size()) {
        SPDLOG_ERROR("size of cam2world or calibration differ from image points");
        return false;
    }
    if (intrinsics.size() < 2) {
        SPDLOG_ERROR("at least two views needed");
        return false;
    }

    vision::Position3D  p3d_init;
    {

        cv::Mat proj1, proj2;
        cv::eigen2cv(create_projection_matrix(cam2world[0], intrinsics[0]), proj1);
        cv::eigen2cv(create_projection_matrix(cam2world[1], intrinsics[1]), proj2);
        cv::Mat cv_result(4, 1, CV_32F);
        std::vector<cv::Point2f> p1, p2;
        p1.push_back(image_point[0]);
        p2.push_back(image_point[1]);

        cv::triangulatePoints(proj1, proj2, p1, p2, cv_result);
        p3d_init = vision::Position3D(cv_result.at<traact::Scalar>(0), cv_result.at<traact::Scalar>(1), cv_result.at<traact::Scalar>(2));
        p3d_init = p3d_init / cv_result.at<traact::Scalar>(3);
    }

    if (image_point.size() == 2) {
        result = p3d_init;
        return true;
    }

    ceres::Problem problem;

    double ceres_result[3];
    ceres_result[0] = p3d_init.x;
    ceres_result[1] = p3d_init.y;
    ceres_result[2] = p3d_init.z;

    for (int i = 0; i < image_point.size(); ++i) {

        //traact::Scalar* tmp = 0;//measurement_for_observation(i);
        // all are single 3d points
        ceres::CostFunction *cost_function =
            PointReprojectionError::Create(image_point[i], cam2world[i], intrinsics[i], i);

        problem.AddResidualBlock(cost_function,
                                 NULL,//new ceres::HuberLoss(1.0), //NULL /* squared loss */,
                                 ceres_result);

    }

    ceres::Solver::Options options;
    options.logging_type = ceres::SILENT;
    //options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
    result = vision::Position3D (ceres_result[0], ceres_result[1], ceres_result[2]);

    if (covariance_output && summary.IsSolutionUsable()) {
        ceres::Covariance::Options options;
        ceres::Covariance covariance(options);

        std::vector<std::pair<const double *, const double *> > covariance_blocks;
        covariance_blocks.push_back(std::make_pair(ceres_result, ceres_result));

        if (!covariance.Compute(covariance_blocks, &problem)) {
            SPDLOG_ERROR("could not compute covariance");

        } else {
            covariance.GetCovarianceBlock(ceres_result, ceres_result, covariance_output);
        }

    }

    return summary.IsSolutionUsable();
}

Eigen::Matrix<traact::Scalar, 3, 4> traact::math::create_projection_matrix(const traact::spatial::Pose6D &cam2world,
                                                                   const vision::CameraCalibration &calibration) {
    Eigen::Matrix4<traact::Scalar> result;

    Eigen::Matrix3<traact::Scalar> intrinsics;

    traact2eigen(calibration, intrinsics);
    Eigen::Matrix4<traact::Scalar> tmp;
    tmp.setIdentity();
    tmp.block<3, 3>(0, 0) = intrinsics;

    result = tmp * cam2world.matrix();

    return Eigen::Matrix<traact::Scalar, 3, 4>(result.block<3, 4>(0, 0));
}

void traact::math::undistort_points(const vision::CameraCalibration &dis_calibration,
                                    const vision::CameraCalibration &undis_calibration,
                                    const std::vector<Eigen::Vector2<traact::Scalar>> &distorted_points,
                                    std::vector<Eigen::Vector2<traact::Scalar>> &undistorted_points) {
    cv::Mat dis_intrinsics, dis_distortion;
    cv::Mat undis_intrinsics, undis_distortion;
    traact2cv(dis_calibration, dis_intrinsics, dis_distortion);
    traact2cv(undis_calibration, undis_intrinsics, undis_distortion);
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    src.resize(distorted_points.size());
    dst.resize(distorted_points.size());
    for (int i = 0; i < distorted_points.size(); ++i) {
        src[i] = cv::Point2d(distorted_points[i].x(), distorted_points[i].y());
    }
    cv::Mat eye3 = cv::Mat::eye(3, 3, CV_32F);
    cv::undistortPoints(src, dst, dis_intrinsics, dis_distortion, eye3, undis_intrinsics);

    undistorted_points.resize(distorted_points.size());
    for (int i = 0; i < distorted_points.size(); ++i) {
        undistorted_points[i] = Eigen::Vector2<traact::Scalar>(dst[i].x, dst[i].y);
        //undistorted_points[i] = Eigen::Vector2<traact::Scalar>(dst[i].x*calibration.fx+calibration.cx,dst[i].y*calibration.fy+calibration.cy);

    }

}


bool traact::math::estimatePose6D(traact::spatial::Pose6D &result, const std::vector<traact::spatial::Pose6D> &cam2world,
                                  const std::vector<vision::CameraCalibration> &intrinsics,
                                  const std::vector<vision::Position2DList> &image_point,
                                  const std::vector<vision::Position3DList> &model, double *covariance_output) {
    if (intrinsics.size() != image_point.size() || cam2world.size() != image_point.size()) {
        SPDLOG_ERROR("size of cam2world or calibration differ from image points");
        return false;
    }

    // should work with only one view
    if (intrinsics.size() < 1) {
        SPDLOG_ERROR("at least one view needed");
        return false;
    }

    ceres::Problem problem;

    double ceres_result[7];
    ceres_result[0] = 1;
    ceres_result[1] = 0;
    ceres_result[2] = 0;
    ceres_result[3] = 0;
    ceres_result[4] = 0;
    ceres_result[5] = 0;
    ceres_result[6] = 0;


    bool init_result{false};
    // try to estimate initial pose with any camera that has at least 5 points
    for (int camera_index = 0; camera_index < image_point.size(); ++camera_index) {
        if(image_point[camera_index].size() > 5){
            spatial::Pose6D init_cam2target;
            init_result = estimate_camera_pose(init_cam2target, image_point[camera_index], intrinsics[camera_index], model[camera_index]);
            if(init_result){
                spatial::Pose6D init_pose = cam2world[camera_index].inverse() * init_cam2target;
                Eigen::Vector3<traact::Scalar> init_pos = init_pose.translation();
                traact::spatial::Rotation3D init_rot(init_pose.rotation());
                ceres_result[0] = init_rot.w();
                ceres_result[1] = init_rot.x();
                ceres_result[2] = init_rot.y();
                ceres_result[3] = init_rot.z();
                ceres_result[4] = init_pos.x();
                ceres_result[5] = init_pos.y();
                ceres_result[6] = init_pos.z();
                break;
            }
        }
    }
    if (!init_result)  {
        SPDLOG_WARN("estimatePose6D: unable to estimate init pose");
    }


    for (int i = 0; i < image_point.size(); ++i) {
        // all are single 3d points
        ceres::CostFunction
            *cost_function = TargetReprojectionErrorFactory::Create(image_point[i], cam2world[i], intrinsics[i], model[i]);

        problem.AddResidualBlock(cost_function,
                                 NULL,//new ceres::HuberLoss(1.0), //NULL /* squared loss */,
                                 ceres_result);

    }

    ceres::Solver::Options options;
    options.logging_type = ceres::SILENT;
    //options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //SPDLOG_INFO(summary.FullReport());
    traact::spatial::Rotation3D
        result_rot = traact::spatial::Rotation3D(ceres_result[0], ceres_result[1], ceres_result[2], ceres_result[3]);
    Eigen::Vector3<traact::Scalar> result_pos = Eigen::Vector3<traact::Scalar>(ceres_result[4], ceres_result[5], ceres_result[6]);
    result.setIdentity();
    result.translate(result_pos);
    result.rotate(result_rot);

    if (covariance_output && summary.IsSolutionUsable()) {
        ceres::Covariance::Options options;
        ceres::Covariance covariance(options);

        std::vector<std::pair<const double *, const double *> > covariance_blocks;
        covariance_blocks.push_back(std::make_pair(ceres_result, ceres_result));

        CHECK(covariance.Compute(covariance_blocks, &problem));

        covariance.GetCovarianceBlock(ceres_result, ceres_result, covariance_output);
    }

    return summary.IsSolutionUsable();
}

traact::Scalar traact::math::reprojectionError(const traact::vision::Position2DList &image_points,
                                               const traact::vision::CameraCalibration &intrinsics,
                                               const traact::vision::Position3DList &camera_to_points) {
    cv::Mat cv_intrinsics;
    cv::Mat cv_distortion;
    traact2cv(intrinsics, cv_intrinsics, cv_distortion);

    vision::Position2DList reprojected_points;
    reprojected_points.reserve(image_points.size());
    cv::Mat tmp(1, 3, CV_32FC1);
    tmp.setTo(0);
    cv::projectPoints(camera_to_points, tmp, tmp, cv_intrinsics, cv_distortion, reprojected_points);

    traact::Scalar error = 0;
    for (int i = 0; i < image_points.size(); ++i) {
        auto diff = reprojected_points[i] - image_points[i];
        error += diff.x*diff.x + diff.y*diff.y;
    }
    error /= image_points.size();

    return std::sqrt(error);
}

traact::Scalar traact::math::reprojectionError(const traact::spatial::Pose6D &camera_to_world,
                                               const traact::vision::Position2DList &image_points,
                                               const traact::vision::CameraCalibration &intrinsics,
                                               const traact::vision::Position3DList &world_to_points) {
    cv::Mat cv_intrinsics, cv_distortion;
    cv::Mat r_vec,t_vec;
    traact2cv(intrinsics, cv_intrinsics, cv_distortion);
    traact2cv(camera_to_world, r_vec, t_vec);

    vision::Position2DList reprojected_points;
    reprojected_points.reserve(image_points.size());
    cv::projectPoints(world_to_points, r_vec, t_vec, cv_intrinsics, cv_distortion, reprojected_points);

    traact::Scalar error = 0;
    for (int i = 0; i < image_points.size(); ++i) {
        auto diff = reprojected_points[i] - image_points[i];
        error += diff.x*diff.x + diff.y*diff.y;
    }
    error /= image_points.size();

    return std::sqrt(error);

}
traact::Scalar traact::math::reprojectionError(const traact::spatial::Pose6D &camera_to_world,
                                               const traact::vision::Position2D &image_points,
                                               const traact::vision::CameraCalibration &intrinsics,
                                               const traact::vision::Position3D &world_to_points) {

    return reprojectionError(camera_to_world, vision::Position2DList {image_points}, intrinsics, vision::Position3DList {world_to_points});
}
traact::Scalar traact::math::reprojectionError(const std::vector<traact::spatial::Pose6D> &camera_to_world,
                                               const std::vector<vision::Position2DList> &image_point,
                                               const std::vector<vision::CameraCalibration> &intrinsics,
                                               const std::vector<vision::Position3DList> &world_to_points) {

    Scalar error{0};
    for (int i = 0; i < camera_to_world.size(); ++i) {
        error += reprojectionError(camera_to_world[i], image_point[i], intrinsics[i], world_to_points[i]);
    }
    error = error / camera_to_world.size();
    return error;
}




