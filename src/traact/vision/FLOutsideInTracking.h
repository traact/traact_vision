/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_FLOUTSIDEINTRACKING_H
#define TRAACTMULTI_FLOUTSIDEINTRACKING_H

#include <traact/spatial.h>
#include "traact/vision.h"

namespace traact::vision {

class Point3DCandidate {
 public:
    std::map<size_t, std::set<size_t> > candidate_points;
    spatial::Position3D point;

    std::map<size_t, size_t> GetGoodCandidates();
    void RemoveCandidate(size_t camera_idx, size_t point_idx);
    void AddCandidate(size_t camera_idx, size_t point_idx);
    bool UsesPoint(size_t camera_idx, size_t point_idx) const;
};

class TrackingCamera {
 public:
    void SetData(const traact::spatial::Pose6D *camera2world,
                 const traact::vision::CameraCalibration *calibration,
                 const traact::spatial::Position2DList *input);
    std::vector<size_t> FindPoints(const spatial::Position3D world2point, traact::Scalar max_distance);
    std::vector<Eigen::ParametrizedLine<traact::Scalar , 3> > rays_;
    spatial::Pose6D camera2world_;
    traact::vision::CameraCalibration calibration_;
    spatial::Position2DList input_;
    spatial::Position2DList *output_;
};

class FLOutsideInTracking {
 public:
    // [3d point idx][camera idx] = point idx
    typedef typename std::vector<std::map<size_t, size_t> > TargetTrackingOutput;
    void SetCountCameras(size_t count);
    void SetData(size_t idx, const spatial::Pose6D *camera2world,
                 const traact::vision::CameraCalibration *calibration,
                 const spatial::Position2DList *input);
    void Compute();
    spatial::Position3DList Get3DPoints();
    /**
     * [camera idx][3d point idx]
     * @return
     */
    std::vector<std::vector<spatial::Position2D> > Get3DPoints2DCorrespondence();

    bool FindTarget(const spatial::Position3DList &model_points, spatial::Pose6D &output,
                    std::vector<spatial::Position2DList *> *output_points,
                    spatial::Position3DList *output_target_points);

    std::vector<TrackingCamera> cameras_;
    traact::spatial::Position3DList current_points3D;
    // [3d point idx][camera idx] = point2d idx
    std::vector<std::map<size_t, size_t> > current_points2D;
 protected:
    bool TestPointAsOrigin(size_t origin_idx, std::map<size_t, size_t> &correspondences);
    bool IsModelPoint(size_t model_idx, size_t point_idx);
    std::map<size_t, size_t> RecursiveFindModel(size_t cur_model_idx, size_t model_count,
                                                std::map<size_t, size_t> correspondences);
    std::vector<std::vector<traact::Scalar> > distances_model_;
    std::vector<std::vector<std::pair<int, traact::Scalar> > > distances_all_;
};
}

#endif //TRAACTMULTI_FLOUTSIDEINTRACKING_H
