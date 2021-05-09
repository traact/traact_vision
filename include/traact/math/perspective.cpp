/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include "perspective.h"
#include <opencv2/core/eigen.hpp>
#include <traact/opencv/OpenCVUtils.h>
#include <traact/util/Logging.h>
#include <opencv2/calib3d.hpp>

#include <ceres/ceres.h>
#include <traact/math/ceres/PointReprojectionError.h>
Eigen::Vector2d
traact::math::reproject_point(const traact::vision::CameraCalibration &intrinsics, const Eigen::Vector3d& point) {
//    Eigen::Vector2d result;
//    Eigen::Matrix<double, 4,4> projection_matrix;
//
//    projection_matrix.col(0) = Eigen::Vector4d(intrinsics.fx, 0,0,0);
//    projection_matrix.col(1) = Eigen::Vector4d(0, intrinsics.fy,0,0);
//    projection_matrix.col(2) = Eigen::Vector4d(intrinsics.cx, intrinsics.cy ,1,0);
//    projection_matrix.col(3) = Eigen::Vector4d(0, 0,0,1);
//
//    Eigen::Vector4d p(point.x(), point.y(), point.z(), 1);
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
    traact2cv(intrinsics,cv_intrinsics, cv_distortion);
    std::vector<cv::Point2f> image_points(1);
    std::vector<cv::Point3f> obj_point(1);
    obj_point[0] = cv::Point3f(point.x(),point.y(), point.z());

    cv::Mat tmp(1,3, CV_32FC1);
    tmp.setTo(0);
    cv::projectPoints(obj_point, tmp, tmp, cv_intrinsics, cv_distortion, image_points );
    return Eigen::Vector2d(image_points[0].x,image_points[0].y);
}

Eigen::Vector2d
traact::math::reproject_point(const Eigen::Affine3d& cam2world, const traact::vision::CameraCalibration &intrinsics, const Eigen::Vector3d& point) {
    Eigen::Vector3d p_tmp;
    p_tmp = cam2world * point;
    return reproject_point(intrinsics, p_tmp);
}

bool traact::math::estimate_camera_pose(spatial::Pose6D &pose_result, const spatial::Position2DList &image_points,
                                        const traact::vision::CameraCalibration &intrinsics,
                                        const spatial::Position3DList &model_points) {

    if(image_points.size() != model_points.size()) {
        SPDLOG_ERROR("size of image and model points differ");
        return false;
    }
    cv::Mat opencv_intrinsics;
    cv::Mat opencv_distortion;
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    std::vector<cv::Point2d> image_points_opencv;
    std::vector<cv::Point3d> model_points_opencv;

    size_t count_points = image_points.size();
    image_points_opencv.resize(count_points);
    model_points_opencv.resize(count_points);

    traact2cv(intrinsics, opencv_intrinsics, opencv_distortion);
    for(int i=0;i<count_points;++i) {
        image_points_opencv[i] = cv::Point2d (image_points[i].x(),image_points[i].y());
        model_points_opencv[i] = cv::Point3d (model_points[i].x(),model_points[i].y(),model_points[i].z());
    }

    bool result = cv::solvePnP(model_points_opencv, image_points_opencv, opencv_intrinsics, opencv_distortion, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE  );
    //bool result = cv::solvePnP(model_points_opencv, image_points_opencv, opencv_intrinsics, opencv_distortion, rvec, tvec, false, cv::SOLVEPNP_P3P  );




    if(!result)
        return false;



    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d R_eigen;
    cv2eigen(R, R_eigen);
    Eigen::Vector3d T_eigen(tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2));


    Eigen::Matrix4d rotate_cs;
    rotate_cs.setIdentity();
    rotate_cs(0,0) = -1;
    rotate_cs(1,1) = -1;
    rotate_cs(3,3) = -1;

    Eigen::Matrix4d Trans;
    Trans.setIdentity();
    Trans.block<3,3>(0,0) = R_eigen;
    Trans.block<3,1>(0,3) = T_eigen;


    //pose_result = Eigen::Affine3d(Trans * rotate_cs);
    pose_result = Eigen::Affine3d(Trans);

    return true;


}

bool traact::math::estimate_3d_point(Eigen::Vector3d &result,  const std::vector<Eigen::Affine3d>& cam2world, const std::vector<vision::CameraCalibration> &intrinsics,
                                     std::vector<Eigen::Vector2d> image_point, double* covariance_output) {
    if(intrinsics.size() != image_point.size() || cam2world.size() != image_point.size()) {
        SPDLOG_ERROR("size of cam2world or calibration differ from image points");
        return false;
    }
    if(intrinsics.size() < 2) {
        SPDLOG_ERROR("at least two views needed");
        return false;
    }

    Eigen::Vector3d p3d_init;
    {

        cv::Mat proj1, proj2;
        cv::eigen2cv(create_projection_matrix(cam2world[0], intrinsics[0]), proj1);
        cv::eigen2cv(create_projection_matrix(cam2world[1], intrinsics[1]), proj2);
        cv::Mat cv_result(4,1,CV_64F);
        std::vector<cv::Point2d> p1, p2;
        p1.push_back(eigen2cv(image_point[0]));
        p2.push_back(eigen2cv(image_point[1]));


        cv::triangulatePoints(proj1, proj2, p1, p2, cv_result);
        p3d_init = Eigen::Vector3d(cv_result.at<double>(0),cv_result.at<double>(1),cv_result.at<double>(2));
        p3d_init = p3d_init / cv_result.at<double>(3);
    }

    if(image_point.size() == 2)
    {
        result = p3d_init;
        return true;
    }

    ceres::Problem problem;

    double ceres_result[3];
    ceres_result[0] = p3d_init.x();
    ceres_result[1] = p3d_init.y();
    ceres_result[2] = p3d_init.z();

    for (int i = 0; i < image_point.size(); ++i) {

       //double* tmp = 0;//measurement_for_observation(i);
       // all are single 3d points
            ceres::CostFunction* cost_function =
                    PointReprojectionError::Create(image_point[i], cam2world[i], intrinsics[i],i);


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
    result = Eigen::Vector3d(ceres_result[0],ceres_result[1],ceres_result[2]);

    if(covariance_output && summary.IsSolutionUsable()){
        ceres::Covariance::Options options;
        ceres::Covariance covariance(options);

        std::vector<std::pair<const double*, const double*> > covariance_blocks;
        covariance_blocks.push_back(std::make_pair(ceres_result, ceres_result));

        CHECK(covariance.Compute(covariance_blocks, &problem));

        covariance.GetCovarianceBlock(ceres_result, ceres_result, covariance_output);
    }


    return summary.IsSolutionUsable();
}

Eigen::Matrix<double, 3, 4> traact::math::create_projection_matrix(const Eigen::Affine3d &cam2world,
                                                       const traact::vision::CameraCalibration calibration) {
    Eigen::Matrix4d result;

    Eigen::Matrix3d intrinsics;

    traact2eigen(calibration, intrinsics);
    Eigen::Matrix4d tmp;
    tmp.setIdentity();
    tmp.block<3,3>(0,0) = intrinsics;

    result = tmp * cam2world.matrix();

    return Eigen::Matrix<double, 3, 4>(result.block<3,4>(0,0));
}

void traact::math::undistort_points(const vision::CameraCalibration& dis_calibration,const vision::CameraCalibration& undis_calibration,
                                    const std::vector<Eigen::Vector2d> &distorted_points,
                                    std::vector<Eigen::Vector2d> &undistorted_points) {
    cv::Mat dis_intrinsics, dis_distortion;
    cv::Mat undis_intrinsics, undis_distortion;
    traact2cv(dis_calibration, dis_intrinsics, dis_distortion);
    traact2cv(undis_calibration, undis_intrinsics, undis_distortion);
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
    src.resize(distorted_points.size());
    dst.resize(distorted_points.size());
    for(int i=0;i<distorted_points.size();++i) {
        src[i] = cv::Point2d(distorted_points[i].x(),distorted_points[i].y());
    }
    cv::Mat eye3 = cv::Mat::eye(3, 3, CV_32F);
    cv::undistortPoints(src, dst, dis_intrinsics,dis_distortion, eye3, undis_intrinsics);

    undistorted_points.resize(distorted_points.size());
    for(int i=0;i<distorted_points.size();++i) {
        undistorted_points[i] = Eigen::Vector2d(dst[i].x,dst[i].y);
        //undistorted_points[i] = Eigen::Vector2d(dst[i].x*calibration.fx+calibration.cx,dst[i].y*calibration.fy+calibration.cy);

    }


}

double traact::math::average_reprojection_error(const Eigen::Affine3d &cam2world,
                                                const spatial::Position2DList &image_points,
                                                const traact::vision::CameraCalibration &intrinsics,
                                                const spatial::Position3DList &model_points) {
    double error = 0;
    for(int i=0;i<image_points.size();++i) {
        auto reprojected_point = reproject_point(cam2world,intrinsics, model_points[i]);
        error += (reprojected_point - image_points[i]).squaredNorm();
    }
    error /= image_points.size();

    return std::sqrt(error);
}


