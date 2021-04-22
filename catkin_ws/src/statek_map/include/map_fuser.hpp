#pragma once

#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "abstract_map.hpp"
#include "laser_scan_map.hpp"

class MapFuser : AbstractMap
{
private:
    // ROS stuff.
    uint32_t msgSeq = 0;
    nav_msgs::OccupancyGrid mapMsg;
    ros::Publisher mapPublisher;

    const double mapUpdateRateMs;
    std::chrono::time_point<std::chrono::high_resolution_clock> previousMapUpdateTime;

    // Odometry compensation.
    double odomOffsetX = 0;
    double odomOffsetY = 0;
    double odomOffsetTheta = 0;

    // Some mapping objects.
    const LaserScanMap &laserScanMap;

    void fuseMaps()
    {
        std::copy(laserScanMap.begin(), laserScanMap.end(), mapMsg.data.begin());
    }

    /**
     * @brief Modified version of Bresenham's line drawing algorithm.
     * 
     * This function will draw a line from start to finish
     * filling all of the pixels with given cellType but also will stop
     * if there is CellType::OBSTACLE_CELL on the way.
     * 
     * @param y0 Start y.
     * @param x0 Start x.
     * @param y1 End y.
     * @param x1 End x.
     * @param cellType Cell type to raycast.
     * @return True if whole line was drawn.
     */
    bool rayTrace(int y0, int x0, int y1, int x1, int8_t cellType)
    {
        int d, dx, dy, ai, bi, xi, yi;
        int x = x0, y = y0;
        if (x0 < x1)
        {
            xi = 1;
            dx = x1 - x0;
        }
        else
        {
            xi = -1;
            dx = x0 - x1;
        }
        if (y0 < y1)
        {
            yi = 1;
            dy = y1 - y0;
        }
        else
        {
            yi = -1;
            dy = y0 - y1;
        }

        if (dx > dy)
        {
            ai = (dy - dx) * 2;
            bi = dy * 2;
            d = bi - dx;

            while (x != x1)
            {
                if (d >= 0)
                {
                    x += xi;
                    y += yi;
                    d += ai;
                }
                else
                {
                    d += bi;
                    x += xi;
                }
                if (this->get(y, x) == CellType::OBSTACLE_CELL)
                    return false;
                this->set(y, x, cellType);
            }
        }
        else
        {
            ai = (dx - dy) * 2;
            bi = dx * 2;
            d = bi - dy;

            while (y != y1)
            {
                if (d >= 0)
                {
                    x += xi;
                    y += yi;
                    d += ai;
                }
                else
                {
                    d += bi;
                    y += yi;
                }
                if (this->get(y, x) == CellType::OBSTACLE_CELL)
                    return false;
                this->set(y, x, cellType);
            }
        }
        return true;
    }

    /**
     * @brief Check whether gap between given coordinates is considered small.
     * 
     * @param y0 Start y.
     * @param x0 Start x.
     * @param y1 End y.
     * @param x1 End x.
     * @return True if this gap is smaller than minimumGapSizeMeters from params.
     */
    bool isSmallGap(int y0, int x0, int y1, int x1) const
    {
        double diffXMeters = toMeters(x1) - toMeters(x0);
        double diffXSquaredMeters = diffXMeters * diffXMeters;
        double diffYMeters = toMeters(y1) - toMeters(y0);
        double diffYSquaredMeters = diffYMeters * diffYMeters;
        return sqrt(diffXSquaredMeters + diffYSquaredMeters) < params.minimumGapSizeMeters;
    }

    /**
     * @brief Close gaps that are smaller than minimumGapSizeMeters from params.
     * 
     * Should be called after map fusion.
     */
    void closeSmallGaps()
    {
        for (int currentPixelY = 0; currentPixelY < params.numCellsPerRowCol; currentPixelY++)
        {
            for (int currentPixelX = 0; currentPixelX < params.numCellsPerRowCol; currentPixelX++)
            {
                // Not an obstacle so nothing to check.
                if (this->get(currentPixelY, currentPixelX) != CellType::OBSTACLE_CELL)
                    continue;

                for (int checkedPixelY = 0; checkedPixelY < params.numCellsPerRowCol; checkedPixelY++)
                {
                    for (int checkedPixelX = 0; checkedPixelX < params.numCellsPerRowCol; checkedPixelX++)
                    {
                        // Not an obstacle so nothing to check.
                        if (this->get(checkedPixelY, checkedPixelX) != CellType::OBSTACLE_CELL)
                            continue;
                        // There can't be gap between two neighbor pixels.
                        if (abs(currentPixelY - checkedPixelY) == 1 || abs(currentPixelX - currentPixelX) == 1)
                            continue;

                        if (this->isSmallGap(currentPixelY, currentPixelX, checkedPixelY, checkedPixelX))
                            this->rayTrace(currentPixelY, currentPixelX, checkedPixelY, checkedPixelX, CellType::OBSTACLE_CELL); // Ray trace obstacle from one pixel to another.
                    }
                }
            }
        }
    }

    /**
     * @brief TODO D:
     */
    void rayTraceFreeCells()
    {
    }

    void generateMap()
    {
        this->reset();
        this->fuseMaps();
        this->closeSmallGaps();
        this->rayTraceFreeCells();
    }

    void publishMap()
    {
        this->generateMap();

        this->mapMsg.header.stamp = ros::Time::now();
        this->mapMsg.header.seq = this->msgSeq;
        this->mapMsg.info.origin.position.x -= params.mapSizeMeters / 2.0 + this->odomOffsetX;
        this->mapMsg.info.origin.position.y -= params.mapSizeMeters / 2.0 + this->odomOffsetY;
        this->mapMsg.info.origin.orientation.z -= this->odomOffsetTheta;
        
        this->mapPublisher.publish(this->mapMsg);

        this->msgSeq++;
    }

protected:
    void reset() override
    {
        std::fill(this->mapMsg.data.begin(), this->mapMsg.data.end(), -1);
    }

public:
    MapFuser(ros::NodeHandle &_nh, const std::string &mapTopic, const std::string &mapFrame, double mapUpdateRate, LaserScanMap &_laserScanMap)
        : mapPublisher(_nh.advertise<nav_msgs::OccupancyGrid>(mapTopic, 1)),
          mapUpdateRateMs(1 / mapUpdateRate * 1000), laserScanMap(_laserScanMap)
    {
        this->mapMsg.info.resolution = params.cellSizeMeters;
        this->mapMsg.info.width = params.numCellsPerRowCol;
        this->mapMsg.info.height = params.numCellsPerRowCol;

        this->mapMsg.header.frame_id = mapFrame;

        this->mapMsg.data = std::vector<int8_t>(params.numCellsPerRowCol * params.numCellsPerRowCol, -1);
    }

    void publish()
    {
        this->publishMap();
    }
};