/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BRUTEFORCEPOSEPNP_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BRUTEFORCEPOSEPNP_H_
#include "traact/vision.h"
namespace traact::vision {

bool testAllCombinations(const Position2DList &points_2d,
                         const Position3DList &points_3d,
                         const CameraCalibration &calibration,
                         spatial::Pose6D &output,
                         Scalar min_error,
                         Scalar max_error,
                         Scalar max_distance,
                         std::vector<size_t> *output_points);
bool tryClusterCombinations(const Position2DList &points2d,
                            const Position3DList &points3d,
                            const CameraCalibration &calibration,
                            spatial::Pose6D &output,
                            Scalar min_error,
                            Scalar max_error,
                            Scalar max_distance,
                            std::vector<size_t> *output_points);

} // traact

#endif //TRAACT_VISION_SRC_TRAACT_VISION_BRUTEFORCEPOSEPNP_H_
