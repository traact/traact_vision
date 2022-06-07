/**
 *   Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com>
 *
 *   License in root folder
**/

#ifndef TRAACTMULTI_OPENCVUTILS_H
#define TRAACTMULTI_OPENCVUTILS_H

#include <traact/vision_datatypes.h>
#include <traact/spatial.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
namespace traact {

static inline cv::Point2d eigen2cv(const Eigen::Vector2d p) {
    return cv::Point2d(p.x(), p.y());
}

static inline cv::Point3d eigen2cv(const Eigen::Vector3d p) {
    return cv::Point3d(p.x(), p.y(), p.z());
}

static inline Eigen::Vector3d cv2eigen(const cv::Point3d p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

static inline void traact2cv(const spatial::Pose6D &pose, cv::Mat &rvec, cv::Mat &tvec) {

    spatial::Position3D trans = pose.translation();
    rvec = cv::Mat(1, 3, CV_64F);
    tvec = cv::Mat(1, 3, CV_64F);
    memcpy(tvec.data, trans.data(), sizeof(double) * 3);
    Eigen::Matrix3d rot_mat = pose.rotation();
    cv::Mat cv_rot_mat;
    cv::eigen2cv(rot_mat, cv_rot_mat);
    cv::Rodrigues(cv_rot_mat, rvec);
}

static inline void cv2traact(const cv::Vec3d &rvec, const cv::Vec3d &tvec, spatial::Pose6D &pose) {

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d R_eigen;
    cv2eigen(R, R_eigen);
    Eigen::Vector3d T_eigen(tvec[0], tvec[1], tvec[2]);

    Eigen::Matrix4d Trans;
    Trans.setIdentity();
    Trans.block<3, 3>(0, 0) = R_eigen;
    Trans.block<3, 1>(0, 3) = T_eigen;

    pose = spatial::Pose6D(Trans);
}

static inline void traact2cv(const vision::CameraCalibration &calibration, cv::Mat &intrinsics, cv::Mat &distortion) {
    cv::Mat opencv_intrinsics(3, 3, cv::DataType<float>::type);
    opencv_intrinsics.at<float>(0, 0) = calibration.fx;
    opencv_intrinsics.at<float>(1, 0) = 0;
    opencv_intrinsics.at<float>(2, 0) = 0;

    opencv_intrinsics.at<float>(0, 1) = 0;
    opencv_intrinsics.at<float>(1, 1) = calibration.fy;
    opencv_intrinsics.at<float>(2, 1) = 0;

    opencv_intrinsics.at<float>(0, 2) = calibration.cx;
    opencv_intrinsics.at<float>(1, 2) = calibration.cy;
    opencv_intrinsics.at<float>(2, 2) = 1;

    intrinsics = opencv_intrinsics;

    size_t count_parameter = calibration.radial_distortion.size() + calibration.tangential_distortion.size();
    cv::Mat opencv_distortion;
    if (count_parameter < 4) {
        opencv_distortion = cv::Mat(4, 1, cv::DataType<float>::type);
        opencv_distortion.at<float>(0) = 0;
        opencv_distortion.at<float>(1) = 0;
        opencv_distortion.at<float>(2) = 0;
        opencv_distortion.at<float>(3) = 0;

    } else {
        opencv_distortion = cv::Mat(count_parameter, 1, cv::DataType<float>::type);

        int parameter_index = 0;
        for (int i = 0; i < 2; ++i) {
            opencv_distortion.at<float>(parameter_index) = calibration.radial_distortion[i];
            ++parameter_index;
        }

        for (int i = 0; i < calibration.tangential_distortion.size(); ++i) {
            opencv_distortion.at<float>(parameter_index) = calibration.tangential_distortion[i];
            ++parameter_index;
        }

        for (int i = 2; i < calibration.radial_distortion.size(); ++i) {
            opencv_distortion.at<float>(parameter_index) = calibration.radial_distortion[i];
            ++parameter_index;
        }

    }

    distortion = opencv_distortion;

}

static inline void cv2traact(vision::CameraCalibration &calibration,
                             const cv::Mat &intrinsics,
                             const cv::Mat &distortion,
                             const cv::Size &size) {
    calibration.fx = intrinsics.at<float>(0, 0);
    calibration.fy = intrinsics.at<float>(1, 1);

    calibration.cx = intrinsics.at<float>(0, 2);
    calibration.cy = intrinsics.at<float>(1, 2);
    calibration.skew = 0;
    calibration.radial_distortion.clear();
    calibration.tangential_distortion.clear();
    calibration.width = size.width;
    calibration.height = size.height;
    if (distortion.rows >= 4) {
        calibration.radial_distortion.push_back(distortion.at<float>(0));
        calibration.radial_distortion.push_back(distortion.at<float>(1));

        calibration.tangential_distortion.push_back(distortion.at<float>(2));
        calibration.tangential_distortion.push_back(distortion.at<float>(3));
    }

    if (distortion.rows > 4) {
        for (int i = 4; i < distortion.rows; ++i) {
            calibration.radial_distortion.push_back(distortion.at<float>(i));
        }
    }

}
}

#endif //TRAACTMULTI_OPENCVUTILS_H
