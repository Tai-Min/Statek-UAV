#pragma once

#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "../abstract_map.hpp"

class LaserScanMap : public AbstractMap
{
public:
    /**
     * @brief Callback called on new lidar data. Constructs new map and transforms it to footprint.
     * @param scan Lidar data.
     */
    void onNewData(const sensor_msgs::LaserScan::ConstPtr &scan);
};