#pragma once

#include <statek_ml/DynamicDetectionArray.h>
#include "../abstract_map.hpp"

class DynamicLegMap : public AbstractMap
{
public:
    /**
     * @brief Callback called on new lidar data. Constructs new map and transforms it to footprint.
     * @param scan Lidar data.
     */
    void onNewData(const statek_ml::DynamicDetectionArray::ConstPtr &detections);
};