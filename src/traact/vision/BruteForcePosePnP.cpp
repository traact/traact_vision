/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "BruteForcePosePnP.h"
#include "traact/math/perspective.h"
#include "traact/opencv/OpenCVUtils.h"

namespace traact::vision {

bool testAllCombinations(const Position2DList &points_2d,
                         const Position3DList &points_3d,
                         const traact::vision::CameraCalibration &calibration,
                         spatial::Pose6D &output,
                         Scalar min_error,
                         Scalar max_error,
                         Scalar max_distance,
                         std::vector<size_t> *output_points) {

    cv::Mat opencv_intrinsics;
    cv::Mat opencv_distortion;
    traact2cv(calibration, opencv_intrinsics, opencv_distortion);

    std::vector<size_t> candidate_index;
    for (int i = 0; i < points_2d.size(); ++i)
        candidate_index.push_back(i);

    std::sort(candidate_index.begin(), candidate_index.end());
    Position2DList cur_image_points;
    cur_image_points.resize(points_3d.size());
    Position2DList final_points;
    bool result = false;
    Scalar current_min_error = std::numeric_limits<Scalar>::max();

    bool end_search = false;
    size_t num_tests = 0;

    do {
        num_tests++;
        Position2D center(0, 0);
        for (int point_index = 0; point_index < points_3d.size(); ++point_index) {
            cur_image_points[point_index] = points_2d[candidate_index[point_index]];
            center = center + cur_image_points[point_index];
        }
        // test if points are close enough to be a target
        // right now mainly to remove reflections away from the target
        center = center / static_cast<float>(points_3d.size());
        bool valid = true;
        for (int point_index = 0; point_index < points_3d.size(); ++point_index) {
            Position2D diff = center - cur_image_points[point_index];
            double distance = cv::norm(diff);
            if (distance > max_distance)
                valid = false;
        }

        if (!valid)
            continue;

        spatial::Pose6D pose;
        bool local_result = math::estimate_camera_pose(pose, cur_image_points,
                                         calibration,
                                         points_3d);
        if (local_result) {

            auto error = traact::math::reprojectionError(pose, cur_image_points, calibration, points_3d);

            if (error < max_error && error < current_min_error) {

                output = pose;
                current_min_error = error;
                if (output_points != nullptr) {
                    *output_points = candidate_index;
                }

                result = true;
                if (error < min_error)
                    end_search = true;
            }
        }
    } while (std::next_permutation(candidate_index.begin(), candidate_index.end()) && !end_search);

    if (result) {
        if (current_min_error > max_error) {
            return false;
        }
        return true;
    } else {
        return false;
    }
}

bool tryClusterCombinations(const Position2DList &points2d,
                            const Position3DList &points3d,
                            const traact::vision::CameraCalibration &calibration,
                            spatial::Pose6D &output,
                            Scalar min_error,
                            Scalar max_error,
                            Scalar max_distance,
                            std::vector<size_t> *output_points) {

    int th_distance = 100; // radius tolerance
    int th2 = th_distance * th_distance; // squared radius tolerance
    std::vector<int> labels;

    int n_labels = partition(points2d, labels, [th2](const cv::Point2d &lhs, const cv::Point2d &rhs) {
        return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < th2;
    });

    SPDLOG_INFO("found labels {0}", n_labels);
    std::vector<Position2DList> labeled_points(n_labels);
    for (int i = 0; i < labels.size(); ++i) {
        labeled_points[labels[i]].push_back(points2d[i]);
    }

    Position2DList found_output_points(points3d.size());

    for (const Position2DList &current_points : labeled_points) {
        SPDLOG_INFO("cluster point count {0}", current_points.size());
        if (current_points.size() < points3d.size() || current_points.size() > 8) {
            SPDLOG_DEBUG("too many points in cluster, abort search for cluster");
            continue;
        }

        bool result = testAllCombinations(current_points,
                                          points3d,
                                          calibration,
                                          output,
                                          min_error,
                                          max_error,
                                          max_distance,
                                          output_points);
        if (result) {
            return true;
        }
    }

    return false;
}
Scalar testCombination(const Position2DList &points_2D,
                       const Position3DList &points_3D,
                       const CameraCalibration &calibration,
                       spatial::Pose6D &output,
                       std::vector<size_t> point_index) {

    Position2DList cur_image_points;
    cur_image_points.resize(points_3D.size());
    for (int index = 0; index < points_3D.size(); ++index) {
        cur_image_points[index] = points_2D[point_index[index]];
    }

    bool local_result = math::estimate_camera_pose(output, cur_image_points, calibration, points_3D);

    if (local_result) {

        double error = traact::math::reprojectionError(output, cur_image_points, calibration, points_3D);

        return error;
    }
    return std::numeric_limits<Scalar>::max();
}

} // traact