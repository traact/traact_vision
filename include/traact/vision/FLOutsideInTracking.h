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

#ifndef TRAACTMULTI_FLOUTSIDEINTRACKING_H
#define TRAACTMULTI_FLOUTSIDEINTRACKING_H

#include <traact/spatial.h>
#include <traact/vision.h>

namespace traact::vision {

    class Point3DCandidate {
    public:
        std::map<std::size_t, std::set<std::size_t> > candidate_points;
        spatial::Position3D point;

        std::map<std::size_t, std::size_t> GetGoodCandidates();
        void RemoveCandidate(std::size_t camera_idx, std::size_t point_idx);
        void AddCandidate(std::size_t camera_idx, std::size_t point_idx);
        bool UsesPoint(std::size_t camera_idx, std::size_t point_idx)const;
    };

    class TrackingCamera {
    public:
        void SetData(const traact::spatial::Pose6D *camera2world,
                     const traact::vision::CameraCalibration *calibration,
                     const traact::spatial::Position2DList *input);
        std::vector<std::size_t> FindPoints(const spatial::Position3D world2point, double max_distance);
        std::vector<Eigen::ParametrizedLine<double, 3> > rays_;
        spatial::Pose6D camera2world_;
        traact::vision::CameraCalibration calibration_;
        spatial::Position2DList input_;
        spatial::Position2DList* output_;
    };

    class FLOutsideInTracking {
    public:
        // [3d point idx][camera idx] = point idx
        typedef typename std::vector<std::map<std::size_t, std::size_t> > TargetTrackingOutput;
        void SetCountCameras(std::size_t count);
        void SetData(std::size_t idx, const spatial::Pose6D *camera2world,
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
        std::vector<std::map<std::size_t, std::size_t> > current_points2D;
    protected:
        bool TestPointAsOrigin(std::size_t origin_idx, std::map<std::size_t, std::size_t>& correspondences);
        bool IsModelPoint(std::size_t model_idx, std::size_t point_idx );
        std::map<std::size_t, std::size_t> RecursiveFindModel(std::size_t cur_model_idx, std::size_t model_count,
                                                              std::map<std::size_t, std::size_t> correspondences);
        std::vector<std::vector<double> > distances_model_;
        std::vector<std::vector<std::pair<int,double> > > distances_all_;
    };
}




#endif //TRAACTMULTI_FLOUTSIDEINTRACKING_H
