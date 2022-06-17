/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_OPENCVUTILS_H
#define TRAACTMULTI_OPENCVUTILS_H

#include <traact/vision_datatypes.h>
#include <traact/spatial.h>
#include <traact/vision.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
namespace traact {

static inline vision::Position2D eigen2cv(const Eigen::Vector2<Scalar> p) {
    return vision::Position2D(p.x(), p.y());
}

static inline vision::Position3D eigen2cv(const Eigen::Vector3<Scalar> p) {
    return vision::Position3D(p.x(), p.y(), p.z());
}

static inline Eigen::Vector3<Scalar> cv2eigen(const vision::Position3D p) {
    return Eigen::Vector3<Scalar>(p.x, p.y, p.z);
}


static inline void traact2cv(const spatial::Pose6D &pose, cv::Mat &rvec, cv::Mat &tvec) {
    if constexpr(std::is_same_v<Scalar, float>){
        rvec = cv::Mat(1, 3, CV_32F);
        tvec = cv::Mat(1, 3, CV_32F);
    } else {
        rvec = cv::Mat(1, 3, CV_64F);
        tvec = cv::Mat(1, 3, CV_64F);
    }
    spatial::Position3D trans = pose.translation();

    memcpy(tvec.data, trans.data(), sizeof(Scalar) * 3);
    Eigen::Matrix3<traact::Scalar> rot_mat = pose.rotation();
    cv::Mat cv_rot_mat;
    cv::eigen2cv(rot_mat, cv_rot_mat);
    cv::Rodrigues(cv_rot_mat, rvec);
}


static inline void cv2traact(const cv::Vec<Scalar, 3> &rvec, const cv::Vec<Scalar, 3> &tvec, spatial::Pose6D &pose) {

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3<traact::Scalar> R_eigen;
    cv2eigen(R, R_eigen);
    Eigen::Vector3<traact::Scalar> T_eigen(tvec[0], tvec[1], tvec[2]);

    Eigen::Matrix4<traact::Scalar> Trans;
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

static inline void cv2traact(const cv::Point2f &cv_point, spatial::Position2D& traact_point){
    traact_point.x() = static_cast<traact::Scalar>(cv_point.x);
    traact_point.y() = static_cast<traact::Scalar>(cv_point.y);
}

static inline void cv2traact(const cv::Point3f &cv_point, spatial::Position3D& traact_point){
    traact_point.x() = static_cast<traact::Scalar>(cv_point.x);
    traact_point.y() = static_cast<traact::Scalar>(cv_point.y);
    traact_point.z() = static_cast<traact::Scalar>(cv_point.z);
}

static inline void cv2traact(const std::vector<cv::Point2f>& cv_points, spatial::Position2DList traact_points) {
    traact_points.resize(cv_points.size());
    for (size_t i=0;i<cv_points.size();++i) {
        cv2traact(cv_points[i], traact_points[i]);
    }
}
static inline void cv2traact(const std::vector<cv::Point3f>& cv_points, spatial::Position3DList traact_points) {
    traact_points.resize(cv_points.size());
    for (size_t i=0;i<cv_points.size();++i) {
        cv2traact(cv_points[i], traact_points[i]);
    }
}
}

#endif //TRAACTMULTI_OPENCVUTILS_H
