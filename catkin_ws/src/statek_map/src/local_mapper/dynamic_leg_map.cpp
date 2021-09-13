#include "../../include/local_mapper/dynamic_leg_map.hpp"
#include <cmath>

void DynamicLegMap::onNewData(const statek_ml::DynamicDetectionArray::ConstPtr &detections)
{
    this->reset();

    for (auto &detection : detections->detections)
    {
        // Get coords of the leg.
        double xMeters = detection.position[0], yMeters = detection.position[1], zMeters = 0;
        this->transformPoint(xMeters, yMeters, zMeters);

        int x, y;
        x = toIndex(xMeters);
        y = toIndex(yMeters);

        // Find direction of movement.
        double vxMeters = detection.velocity[0], vyMeters = detection.velocity[1], vzMeters = 0;
        this->transformPoint(vxMeters, vyMeters, vzMeters);

        // Draw obstacles from legs to edge of the map
        // based on it's movement vector.
        int vxSign = vxMeters / abs(vxMeters);
        int vySign = vyMeters / abs(vyMeters);

        double vx = vxMeters / (vxMeters + vyMeters);
        double vy = vyMeters / (vxMeters + vyMeters);

        double nextX = x;
        double nextY = y;
        while (isValidPoint(nextY, nextX))
        {
            this->set(nextY, nextX, CellType::OBSTACLE_CELL);
            nextX += vx * vxSign;
            nextY += vy * vySign;
        }
    }
}