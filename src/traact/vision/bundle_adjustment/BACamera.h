/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BACAMERA_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BACAMERA_H_

#include <traact/vision.h>
#include "BAData.h"

namespace traact::vision::bundle_adjustment {

    class BACamera : public BAData<std::vector<Eigen::Vector2d>>{
     public:
        typedef typename std::shared_ptr<BACamera> Ptr;

        BACamera(const std::string &name);

        bool isStaticPosition() const;

        void setStaticPosition(bool staticPosition);

        bool isStaticRotation() const;

        void setStaticRotation(bool staticRotation);

        const vision::CameraCalibration &getIntrinsic() const;

        void setIntrinsic(const vision::CameraCalibration &intrinsic);

        const spatial::Pose6D &getExtrinsic() const;

        void setExtrinsic(const spatial::Pose6D &extrinsic);

        const std::string &getResultfile() const;

        void setResultfile(const std::string &resultfile);

        std::string toString();


        void tryFindingImagePoints(const std::vector<Eigen::Vector3d>& world_points, std::vector<Eigen::Vector2d>& image_points, const vision::KeyPointList& all_points);

     protected:
        std::string name_;
        bool static_position;
        bool static_rotation;
        CameraCalibration intrinsic_;
        spatial::Pose6D extrinsic_;
        std::string resultfile_;


    };
}

#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BACAMERA_H_
