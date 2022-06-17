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

Eigen::Vector2<traact::Scalar>
traact::math::reproject_point(const traact::vision::CameraCalibration &intrinsics, const Eigen::Vector3<traact::Scalar> &point) {
//    Eigen::Vector2<traact::Scalar> result;
//    Eigen::Matrix<traact::Scalar, 4,4> projection_matrix;
//
//    projection_matrix.col(0) = Eigen::Vector4<traact::Scalar>(intrinsics.fx, 0,0,0);
//    projection_matrix.col(1) = Eigen::Vector4<traact::Scalar>(0, intrinsics.fy,0,0);
//    projection_matrix.col(2) = Eigen::Vector4<traact::Scalar>(intrinsics.cx, intrinsics.cy ,1,0);
//    projection_matrix.col(3) = Eigen::Vector4<traact::Scalar>(0, 0,0,1);
//
//    Eigen::Vector4<traact::Scalar> p(point.x(), point.y(), point.z(), 1);
//
//    auto tmp = projection_matrix * p;
//
//    result.x() = tmp.x() /tmp.z();
//    result.y() = tmp.y() /tmp.z();
//
//
//    //return std::move(result);
//    return result;
    cv::Mat cv_intrinsics;
    cv::Mat cv_distortion;
    traact2cv(intrinsics, cv_intrinsics, cv_distortion);
    std::vector<cv::Point2f> image_points(1);
    std::vector<cv::Point3f> obj_point(1);
    obj_point[0] = cv::Point3f(point.x(), point.y(), point.z());

    cv::Mat tmp(1, 3, CV_32FC1);
    tmp.setTo(0);
    cv::projectPoints(obj_point, tmp, tmp, cv_intrinsics, cv_distortion, image_points);
    return Eigen::Vector2<traact::Scalar>(image_points[0].x, image_points[0].y);
}

Eigen::Vector2<traact::Scalar>
traact::math::reproject_point(const traact::spatial::Pose6D &cam2world,
                              const traact::vision::CameraCalibration &intrinsics,
                              const Eigen::Vector3<traact::Scalar> &point) {
    Eigen::Vector3<traact::Scalar> p_tmp;
    p_tmp = cam2world * point;
    return reproject_point(intrinsics, p_tmp);
}

bool traact::math::estimate_camera_pose(spatial::Pose6D &pose_result, const spatial::Position2DList &image_points,
                                        const traact::vision::CameraCalibration &intrinsics,
                                        const spatial::Position3DList &model_points) {

    if (image_points.size() != model_points.size()) {
        SPDLOG_ERROR("size of image and model points differ");
        return false;
    }
    cv::Mat opencv_intrinsics;
    cv::Mat opencv_distortion;
    cv::Mat tvec(3, 1, cv::DataType<traact::Scalar>::type);
    cv::Mat rvec(3, 1, cv::DataType<traact::Scalar>::type);
    std::vector<cv::Point2d> image_points_opencv;
    std::vector<cv::Point3d> model_points_opencv;

    size_t count_points = image_points.size();
    image_points_opencv.resize(count_points);
    model_points_opencv.resize(count_points);

    traact2cv(intrinsics, opencv_intrinsics, opencv_distortion);
    for (int i = 0; i < count_points; ++i) {
        image_points_opencv[i] = cv::Point2d(image_points[i].x(), image_points[i].y());
        model_points_opencv[i] = cv::Point3d(model_points[i].x(), model_points[i].y(), model_points[i].z());
    }

    bool result = cv::solvePnP(model_points_opencv,
                               image_points_opencv,
                               opencv_intrinsics,
                               opencv_distortion,
                               rvec,
                               tvec,
                               false,
                               cv::SOLVEPNP_ITERATIVE);
    //bool result = cv::solvePnP(model_points_opencv, image_points_opencv, opencv_intrinsics, opencv_distortion, rvec, tvec, false, cv::SOLVEPNP_P3P  );




    if (!result)
        return false;

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3<traact::Scalar> R_eigen;
    cv2eigen(R, R_eigen);
    Eigen::Vector3<traact::Scalar> T_eigen(tvec.at<traact::Scalar>(0), tvec.at<traact::Scalar>(1), tvec.at<traact::Scalar>(2));

    Eigen::Matrix4<traact::Scalar> rotate_cs;
    rotate_cs.setIdentity();
    rotate_cs(0, 0) = -1;
    rotate_cs(1, 1) = -1;
    rotate_cs(3, 3) = -1;

    Eigen::Matrix4<traact::Scalar> Trans;
    Trans.setIdentity();
    Trans.block<3, 3>(0, 0) = R_eigen;
    Trans.block<3, 1>(0, 3) = T_eigen;


    //pose_result = traact::spatial::Pose6D(Trans * rotate_cs);
    pose_result = traact::spatial::Pose6D(Trans);

    return true;

}

bool traact::math::estimate_3d_point(Eigen::Vector3<traact::Scalar> &result,
                                     const std::vector<traact::spatial::Pose6D> &cam2world,
                                     const std::vector<vision::CameraCalibration> &intrinsics,
                                     const std::vector<Eigen::Vector2<traact::Scalar>> &image_point,
                                     double *covariance_output) {
    if (intrinsics.size() != image_point.size() || cam2world.size() != image_point.size()) {
        SPDLOG_ERROR("size of cam2world or calibration differ from image points");
        return false;
    }
    if (intrinsics.size() < 2) {
        SPDLOG_ERROR("at least two views needed");
        return false;
    }

    Eigen::Vector3<traact::Scalar> p3d_init;
    {

        cv::Mat proj1, proj2;
        cv::eigen2cv(create_projection_matrix(cam2world[0], intrinsics[0]), proj1);
        cv::eigen2cv(create_projection_matrix(cam2world[1], intrinsics[1]), proj2);
        cv::Mat cv_result(4, 1, CV_64F);
        std::vector<cv::Point2d> p1, p2;
        p1.push_back(eigen2cv(image_point[0]));
        p2.push_back(eigen2cv(image_point[1]));

        cv::triangulatePoints(proj1, proj2, p1, p2, cv_result);
        p3d_init = Eigen::Vector3<traact::Scalar>(cv_result.at<traact::Scalar>(0), cv_result.at<traact::Scalar>(1), cv_result.at<traact::Scalar>(2));
        p3d_init = p3d_init / cv_result.at<traact::Scalar>(3);
    }

    if (image_point.size() == 2) {
        result = p3d_init;
        return true;
    }

    ceres::Problem problem;

    double ceres_result[3];
    ceres_result[0] = p3d_init.x();
    ceres_result[1] = p3d_init.y();
    ceres_result[2] = p3d_init.z();

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
    result = Eigen::Vector3<traact::Scalar>(ceres_result[0], ceres_result[1], ceres_result[2]);

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

traact::Scalar traact::math::average_reprojection_error(const traact::spatial::Pose6D &cam2world,
                                                const spatial::Position2DList &image_points,
                                                const traact::vision::CameraCalibration &intrinsics,
                                                const spatial::Position3DList &model_points) {
    traact::Scalar error = 0;
    for (int i = 0; i < image_points.size(); ++i) {
        auto reprojected_point = reproject_point(cam2world, intrinsics, model_points[i]);
        error += (reprojected_point - image_points[i]).squaredNorm();
    }
    error /= image_points.size();

    return std::sqrt(error);
}

bool traact::math::estimate_3d_pose(traact::spatial::Pose6D &result, const std::vector<traact::spatial::Pose6D> &cam2world,
                                    const std::vector<vision::CameraCalibration> &intrinsics,
                                    const std::vector<spatial::Position2DList> &image_point,
                                    const spatial::Position3DList &model, double *covariance_output) {
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

    spatial::Pose6D init_cam2target;
    bool init_result = estimate_camera_pose(init_cam2target, image_point[0], intrinsics[0], model);
    if (init_result) {
        spatial::Pose6D init_pose = cam2world[0].inverse() * init_cam2target;
        Eigen::Vector3<traact::Scalar> init_pos = init_pose.translation();
        traact::spatial::Rotation3D init_rot(init_pose.rotation());
        ceres_result[0] = init_rot.w();
        ceres_result[1] = init_rot.x();
        ceres_result[2] = init_rot.y();
        ceres_result[3] = init_rot.z();
        ceres_result[4] = init_pos.x();
        ceres_result[5] = init_pos.y();
        ceres_result[6] = init_pos.z();

    } else {
        SPDLOG_WARN("estimate_3d_pose: unable to estimate init pose");
    }

    for (int i = 0; i < image_point.size(); ++i) {

        //traact::Scalar* tmp = 0;//measurement_for_observation(i);
        // all are single 3d points
        ceres::CostFunction
            *cost_function = TargetReprojectionErrorFactory::Create(image_point[i], cam2world[i], intrinsics[i], model);

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

bool traact::math::estimate_3d_pose(traact::spatial::Pose6D &result, const traact::spatial::Position3DList &src,
                                    const traact::spatial::Position3DList &dst, double *covariance_output) {
    if (src.size() != dst.size()) {
        SPDLOG_ERROR("size of src points differs from src points");
        return false;
    }

    // should work with only one view
//    if(intrinsics.size() < 2) {
//        SPDLOG_ERROR("at least two views needed");
//        return false;
//    }



    ceres::Problem problem;

    double ceres_result[7];
    ceres_result[0] = 1;
    ceres_result[1] = 0;
    ceres_result[2] = 0;
    ceres_result[3] = 0;
    ceres_result[4] = 0;
    ceres_result[5] = 0;
    ceres_result[6] = 0;
    ceres_result[7] = 0;

    ceres::CostFunction *cost_function = DistanceError3D3DFactory::Create(src, dst);

    problem.AddResidualBlock(cost_function,
                             NULL,//new ceres::HuberLoss(1.0), //NULL /* squared loss */,
                             ceres_result);

    ceres::Solver::Options options;
    options.logging_type = ceres::SILENT;
    //options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
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

traact::Scalar
traact::math::reprojection_error(const traact::spatial::Pose6D &cam2world, const traact::spatial::Position2D &image_points,
                                 const traact::vision::CameraCalibration &intrinsics,
                                 const traact::spatial::Position3D &model_points) {
    auto reprojected_point = reproject_point(cam2world, intrinsics, model_points);
    return (reprojected_point - image_points).norm();
}



