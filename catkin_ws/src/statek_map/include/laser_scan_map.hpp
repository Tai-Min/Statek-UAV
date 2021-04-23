#pragma once

#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "abstract_map.hpp"

class LaserScanMap : public AbstractMap
{
public:
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

            unsigned int x = toIndex(ray * cos(currentAngle));
            unsigned int y = toIndex(ray * sin(currentAngle));

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