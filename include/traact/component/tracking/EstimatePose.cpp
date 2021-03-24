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

#include <rttr/registration>


#include <traact/traact.h>
#include <traact/vision.h>
#include <opencv2/videoio.hpp>
#include <traact/component/vision/BasicVisionPattern.h>
#include <traact/vision/UndistortionHelper.h>
#include <traact/math/perspective.h>
#include <traact/spatial.h>
#include <iostream>
#include <fstream>

namespace traact::component::vision {

    class EstimatePose : public Component {
    public:
        explicit EstimatePose(const std::string &name) : Component(name,
                                                                           traact::component::ComponentType::Functional) {
        }

        traact::pattern::Pattern::Ptr GetPattern()  const {


            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern =
                    std::make_shared<traact::pattern::spatial::SpatialPattern>("EstimatePose", unlimited);

            pattern->addConsumerPort("input", traact::spatial::Position2DListHeader::MetaType);
            pattern->addConsumerPort("input_model", traact::spatial::Position3DListHeader::MetaType);
            pattern->addConsumerPort("input_calibration", traact::vision::CameraCalibrationHeader::MetaType);

            pattern->addProducerPort("output", traact::spatial::Pose6DHeader::MetaType);
            pattern->addProducerPort("output_points", traact::spatial::Position2DListHeader::MetaType);

            pattern->addParameter("maxPointDistance", 100.0);
            pattern->addParameter("minError", 1.0);
            pattern->addParameter("maxError", 1.0);
            pattern->addParameter("forceZFaceCamera", true);




            return pattern;
        }

        bool configure(const nlohmann::json &parameter, buffer::GenericComponentBufferConfig *data) override {
            pattern::setValueFromParameter(parameter, "maxPointDistance", maxPointDistance_, 100);
            pattern::setValueFromParameter(parameter, "minError", minError_, 1.0);
            pattern::setValueFromParameter(parameter, "maxError", maxError_, 1.0);
            pattern::setValueFromParameter(parameter, "forceZFaceCamera", forceZFaceCamera_, true);

            return true;
        }

        bool processTimePoint(buffer::GenericComponentBuffer &data) override {
            using namespace traact::vision;
            const auto& points2d = data.getInput<spatial::Position2DList, spatial::Position2DListHeader>(0);
            const auto& points3d = data.getInput<spatial::Position3DList, spatial::Position3DListHeader>(1);
            const auto& calibration = data.getInput<CameraCalibrationHeader::NativeType, CameraCalibrationHeader>(2);

            auto& output = data.getOutput<traact::spatial::Pose6DHeader::NativeType, traact::spatial::Pose6DHeader>(0);
            auto& output_points = data.getOutput<spatial::Position2DList, traact::spatial::Position2DListHeader>(1);

            if(points2d.size() < points3d.size())
                return false;



            std::vector<std::size_t> candidate_index;
            for(int i=0;i<points2d.size();++i)
                candidate_index.push_back(i);

            std::sort(candidate_index.begin(), candidate_index.end());
            spatial::Position2DList cur_image_points;
            cur_image_points.resize(points3d.size());
            std::vector<Eigen::Vector2d> final_points;

            bool result = false;
            double min_error = std::numeric_limits<double>::max();

            bool endSearch = false;
            std::size_t num_tests = 0;

            do {
                num_tests++;
                Eigen::Vector2d center = Eigen::Vector2d::Zero();
                for(int point_index = 0;point_index < points3d.size();++point_index){
                    cur_image_points[point_index] = points2d[candidate_index[point_index]];
                    center = center + cur_image_points[point_index];
                }
                // test if points are close enough to be a target
                // right now mainly to remove reflections away from the target
                center = center / points3d.size();
                bool valid = true;
                for(int point_index = 0;point_index < points3d.size();++point_index){
                    Eigen::Vector2d diff = center - cur_image_points[point_index];
                    double distance = diff.norm();
                    if(distance > maxPointDistance_)
                        valid = false;
                }

                if(!valid)
                    continue;


                Eigen::Affine3d pose;
                bool local_result = traact::math::estimate_camera_pose(pose,cur_image_points,calibration,points3d);
                if(local_result) {
                    double error = traact::math::average_reprojection_error(pose,cur_image_points, calibration, points3d);

                    if(error < maxError_ && error < min_error){

                        if(forceZFaceCamera_){
                            auto p0 = traact::math::reproject_point(pose, calibration,
                                                                    Eigen::Vector3d(0, 0, 0));
                            auto pz = traact::math::reproject_point(pose, calibration,
                                                                    Eigen::Vector3d(0, 0, 1));



                            double diff = p0.y() - pz.y();
                            if(diff < 0)
                                continue;
                        }

                        output = pose;
                        min_error = error;
                        output_points = cur_image_points;
                        result = true;
                        if(error < minError_)
                            endSearch = true;
                    }
                }
            } while(std::next_permutation(candidate_index.begin(), candidate_index.end()) && !endSearch);

            if(result){

                if(min_error > maxError_){
                    SPDLOG_INFO("found pose but rejected, final error bigger then max error {2}: {0} in {1} tests", min_error, num_tests, maxError_);
                    return false;
                }
                SPDLOG_INFO("found pose final error: {0} in {1} tests", min_error, num_tests);
                return true;


            } else {
                SPDLOG_INFO("no pose found final error: {0} in {1} tests", min_error, num_tests);
                return false;
            }
        }


    private:
        double maxPointDistance_;
        double minError_;
        double maxError_;
        bool forceZFaceCamera_;


    RTTR_ENABLE(Component)

    };

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::EstimatePose>("EstimatePose").constructor<std::string>()();
}