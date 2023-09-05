/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BADATA_H_
#define TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BADATA_H_

#include <traact/datatypes.h>
#include <map>
#include <set>

namespace traact::vision::bundle_adjustment {

template<class T>
class BAData {
 public:
    BAData() = default;
    virtual ~BAData() = default;
    void SetMeasurement(Timestamp ts, T world_points) {
        data_[ts] = world_points;
    }


    bool HasMeasurement(Timestamp ts) {
        auto result = data_.find(ts);
        if(result == data_.end())
            return false;
        return !result->second.empty();
    }

    T GetMeasurement(Timestamp ts) {
        return data_.at(ts);
    }

    std::set<Timestamp> GetAllTimestamps() {
        std::set<Timestamp> result;
        for(const auto& ts_data : data_){
            result.emplace(ts_data.first);
        }
        return result;
    }



 protected:
    std::map<Timestamp,T> data_;

};
}

#endif //TRAACT_VISION_SRC_TRAACT_VISION_BUNDLE_ADJUSTMENT_BADATA_H_
