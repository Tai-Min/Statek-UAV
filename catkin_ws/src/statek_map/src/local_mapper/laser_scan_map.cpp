#include "../../include/local_mapper/laser_scan_map.hpp"
#include <cmath>

void LaserScanMap::onNewData(const sensor_msgs::LaserScan::ConstPtr &scan)
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

        this->transformPoint(xMeters, yMeters, zMeters);

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