#pragma once

#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "abstract_map.hpp"
#include <iostream>
class LaserScanMap : public AbstractMap
{
public:
    /**
     * @brief Callback called on new lidar data. Constructs new map and transforms it to footprint.
     * @param scan Lidar data.
     */
    void onNewData(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        this->reset();

        double currentAngle = scan->angle_min;
        double increment = scan->angle_increment;

        for (unsigned int i = 0; i < scan->ranges.size(); i++)
        {
            double ray = scan->ranges[i];
            // 0 means there is no obstacle detected.
            if (ray == 0)
            {
                currentAngle += increment;
                continue;
            }

            double xMeters = ray * cos(currentAngle);
            double yMeters = ray * sin(currentAngle);
            double zMeters = 0; // Unused but required for translation.

            this->transformToFootprint(xMeters, yMeters, zMeters);

            int x = toIndex(xMeters);
            int y = toIndex(yMeters);

            // Out of bounds.
            if (x < 0 || y < 0 || x >= params.numCellsPerRowCol || y >= params.numCellsPerRowCol)
            {
                currentAngle += increment;
                continue;
            }

            this->set(y, x, CellType::OBSTACLE_CELL);
            currentAngle += increment;
        }
    }
};