#pragma once

#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "abstract_map.hpp"
#include "laser_scan_map.hpp"

class MapFuser : AbstractMap
{
private:
    // ROS stuff.
    uint32_t msgSeq = 0;
    nav_msgs::OccupancyGrid mapMsg;
    tf::StampedTransform stampedTransform;

    const int mapUpdateRateMs;
    std::chrono::time_point<std::chrono::high_resolution_clock> previousMapUpdateTime;

    // Odometry memory for compensation.
    double previousOdomX = 0;
    double previousOdomY = 0;
    double previousOdomTheta = 0;

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

                if (this->get(y, x) == CellType::OBSTACLE_CELL || this->get(y, x) == CellType::FILLED_GAP)
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

                if (this->get(y, x) == CellType::OBSTACLE_CELL || this->get(y, x) == CellType::FILLED_GAP)
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

        return (diffXSquaredMeters + diffYSquaredMeters) < params.minimumGapSizeMetersSquared;
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

                for (int checkedPixelY = currentPixelY; checkedPixelY < params.numCellsPerRowCol; checkedPixelY++)
                {
                    for (int checkedPixelX = 0; checkedPixelX < params.numCellsPerRowCol; checkedPixelX++)
                    {
                        // Not an obstacle so nothing to check.
                        if (this->get(checkedPixelY, checkedPixelX) != CellType::OBSTACLE_CELL)
                            continue;

                        // There can't be gap between two neighbor pixels.
                        if (abs(currentPixelY - checkedPixelY) == 1 && abs(currentPixelX - currentPixelX) == 1)
                            continue;

                        // Ray trace filled gap from one pixel to another.
                        if (this->isSmallGap(currentPixelY, currentPixelX, checkedPixelY, checkedPixelX))
                            this->rayTrace(currentPixelY, currentPixelX, checkedPixelY, checkedPixelX, CellType::FILLED_GAP);
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
        // Ray trace free cells from center of the map to edges.
        for (int currentPixelY = 0; currentPixelY < params.numCellsPerRowCol; currentPixelY++)
        {
            // Fill (y = first and every x) and (y = last and every x).
            if (currentPixelY == 0 || currentPixelY == params.numCellsPerRowCol - 1)
            {
                for (int currentPixelX = 0; currentPixelX < params.numCellsPerRowCol; currentPixelX++)
                {

                    this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0, currentPixelY, currentPixelX, CellType::FREE_CELL); // Ray trace obstacle from one pixel to another.
                }
            }
            // fill (every other y and x = first) and (every other y and x = last).
            else
            {
                this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0, currentPixelY, 0, CellType::FREE_CELL); // Ray trace obstacle from one pixel to another.
                this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0, currentPixelY, params.numCellsPerRowCol - 1, CellType::FREE_CELL);
            }
        }
    }

    void updateMap()
    {
        this->reset();
        this->fuseMaps();
        this->closeSmallGaps();
        this->rayTraceFreeCells();

        // Map generated so reset offset.
        this->odomOffsetX = 0;
        this->odomOffsetY = 0;
        this->odomOffsetTheta = 0;

        // Fill message with required data.
        this->mapMsg.header.stamp = ros::Time::now();
        this->mapMsg.header.seq = this->msgSeq;
        this->mapMsg.info.origin.position.x = -params.mapSizeMeters / 2.0;
        this->mapMsg.info.origin.position.y = -params.mapSizeMeters / 2.0;
        this->mapMsg.info.origin.orientation.z = this->odomOffsetTheta;

        this->msgSeq++;
    }

protected:
    void reset() override
    {
        std::fill(this->mapMsg.data.begin(), this->mapMsg.data.end(), CellType::UNKNOWN_CELL);
    }

    std::vector<int8_t>::const_iterator end() const override
    {
        return this->mapMsg.data.end();
    }

    int8_t operator[](const unsigned int index) const override
    {
        return this->mapMsg.data[index];
    }

    void set(unsigned int y, unsigned int x, int8_t val) override
    {
        this->mapMsg.data[y * params.numCellsPerRowCol + x] = val;
    }

    int8_t get(unsigned int y, unsigned int x) const override
    {
        return this->mapMsg.data[y * params.numCellsPerRowCol + x];
    }

public:
    MapFuser(const std::string &odomFrame, const std::string &mapFrame, int _mapUpdateRateMs, LaserScanMap &_laserScanMap)
        : mapUpdateRateMs(_mapUpdateRateMs), laserScanMap(_laserScanMap)
    {
        this->mapMsg.info.resolution = params.cellSizeMeters;
        this->mapMsg.info.width = params.numCellsPerRowCol;
        this->mapMsg.info.height = params.numCellsPerRowCol;

        this->mapMsg.header.frame_id = mapFrame;

        this->resize();

        tf::Transform transform;
        this->stampedTransform.child_frame_id_ = odomFrame;
        this->stampedTransform.frame_id_ = mapFrame;
    }

    void onNewOdom(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // Convert to tf quaternion.
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

        // Convert to RPY angles.
        double temp1, temp2, theta;
        tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);

        double travelledDistanceX = odom->pose.pose.position.x - this->previousOdomX;
        double travelledDistanceY = odom->pose.pose.position.y - this->previousOdomY;
        double travelledDistanceTheta = theta - this->previousOdomTheta;

        this->odomOffsetX += travelledDistanceX;
        this->odomOffsetY += travelledDistanceY;
        this->odomOffsetTheta += travelledDistanceTheta;

        this->previousOdomX = odom->pose.pose.position.x;
        this->previousOdomY = odom->pose.pose.position.y;
        this->previousOdomTheta = theta;
    }

    void resize() override
    {
        this->mapMsg.data.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
        this->reset();
    }

    const tf::StampedTransform &getTransform()
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(this->previousOdomX + this->odomOffsetX, -this->previousOdomY + this->odomOffsetY, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, -this->previousOdomTheta + this->odomOffsetTheta - M_PI));

        this->stampedTransform.stamp_ = ros::Time::now();
        this->stampedTransform.setData(transform);

        return this->stampedTransform;
    }

    const nav_msgs::OccupancyGrid &getMapMsg()
    {
        return this->mapMsg;
    }

    bool tryUpdateMap()
    {
        auto now = std::chrono::system_clock::now();
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->previousMapUpdateTime);
        unsigned int ms = milliseconds.count();

        if (ms > this->mapUpdateRateMs)
        {
            this->updateMap();

            this->previousMapUpdateTime = now;
            return true;
        }
        return false;
    }
};