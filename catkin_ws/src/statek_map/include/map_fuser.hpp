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
    nav_msgs::OccupancyGrid mapMsg;                   //!< Stores map and it's stuff.
    geometry_msgs::TransformStamped stampedTransform; //!< Stores transform from map to robot footprint.

    const int mapUpdateRateMs;                                                         //!< Time in ms that should pass between map updates.
    std::chrono::time_point<std::chrono::high_resolution_clock> previousMapUpdateTime; //!< Stores time of previous map update.

    // Odometry memory for compensation.
    double latestOdomX = 0;     //!< Latest odometry reading. Required for tf.
    double latestOdomY = 0;     //!< Latest odometry reading. Required for tf.
    double latestOdomTheta = 0; //!< Latest odometry reading. Required for tf.

    // Odometry compensation.
    double odomOffsetX = 0;     //!< Accomodate movement in x direction between map updates.
    double odomOffsetY = 0;     //!< Accomodate movement in y direction between map updates.
    double odomOffsetTheta = 0; //!< Accomodate rotation between map updates.

    // Some mapping objects.
    const std::vector<std::reference_wrapper<AbstractMap>> maps;

    /**
     * @brief Check whether given coordinates are between 0 (inclusive) and sizeX / sizeY (exclusive).
     * @return True if both y and x are in range.
     */
    static bool isInRange(int y, int x, int sizeY, int sizeX)
    {
        if (y < 0 || y >= sizeY)
            return false;

        if (x < 0 || x >= sizeX)
            return false;
        return true;
    }

    /**
     * @brief Perform sensor fusion.
     */
    void fuseMaps()
    {
        std::copy(maps[0].get().begin(), maps[0].get().end(), mapMsg.data.begin());
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
     * @param sizeX Number of cells along X axis.
     * @param sizeY Number of cells along Y axis.
     * @param cellType Cell type to raycast.
     * @param bold Whether the line should be bold.
     * @param stopOnFilled Whether the algorithm should also stop on CellType::FILLED_GAP.
     * @return True if whole line was drawn. False if stopped on CellType::OBSTACLE_CELL or CellType::FILLED_GAP of stopOnFilled = true.
     */
    bool rayTrace(int y0, int x0, int y1, int x1, int sizeX, int sizeY, int8_t cellType, bool bold = false, bool stopOnFilled = false)
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

        // First free cell should be drawn.
        if(cellType == FREE_CELL){
            this->set(y, x, cellType);
        }

        if (dx > dy)
        {
            ai = (dy - dx) * 2;
            bi = dy * 2;
            d = bi - dx;

            while (x != x1)
            {
                bool makeBold = false;
                if (d >= 0)
                {
                    x += xi;
                    y += yi;
                    d += ai;
                    makeBold = true;
                }
                else
                {
                    d += bi;
                    x += xi;
                }

                // Draw additional pixels to make line bolder if requested.
                if (bold && makeBold)
                {
                    int boldX = x;
                    int boldY = y - yi;
                    if (isInRange(boldY, boldX, sizeY, sizeX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                        this->set(boldY, boldX, cellType);

                    boldX = x - xi;
                    boldY = y;
                    if (isInRange(boldY, boldX, sizeY, sizeX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                        this->set(boldY, boldX, cellType);
                }

                // Stop on obstacle.
                if (this->get(y, x) == CellType::OBSTACLE_CELL)
                    return false;

                // Stop on filled gap if requested.
                if (this->get(y, x) == CellType::FILLED_GAP && stopOnFilled)
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
                bool makeBold = false;
                if (d >= 0)
                {
                    x += xi;
                    y += yi;
                    d += ai;
                    makeBold = true;
                }
                else
                {
                    d += bi;
                    y += yi;
                }

                // Draw additional pixels to make line bolder if requested.
                if (bold && makeBold)
                {
                    int boldX = x;
                    int boldY = y - yi;
                    if (isInRange(boldY, boldX, sizeY, sizeX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                        this->set(boldY, boldX, cellType);

                    boldX = x - xi;
                    boldY = y;
                    if (isInRange(boldY, boldX, sizeY, sizeX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                        this->set(boldY, boldX, cellType);
                }

                // Stop on obstacle.
                if (this->get(y, x) == CellType::OBSTACLE_CELL)
                    return false;

                // Stop on filled gap if requested.
                if (this->get(y, x) == CellType::FILLED_GAP && stopOnFilled)
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

                        // Ray trace filled gap from one pixel to another.
                        if (this->isSmallGap(currentPixelY, currentPixelX, checkedPixelY, checkedPixelX))
                            this->rayTrace(currentPixelY, currentPixelX,
                                           checkedPixelY, checkedPixelX,
                                           params.numCellsPerRowCol, params.numCellsPerRowCol,
                                           CellType::FILLED_GAP, true, false);
                    }
                }
            }
        }
    }

    /**
     * @brief Set all cells visible by the robot to CellType::FREE_CELL. 
     * 
     * Should be only called in updateMap() after all of the sensor data had been fused.
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

                    this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0,
                                   currentPixelY, currentPixelX,
                                   params.numCellsPerRowCol, params.numCellsPerRowCol,
                                   CellType::FREE_CELL, false, true); // Ray trace obstacle from one pixel to another.
                }
            }
            // fill (every other y and x = first) and (every other y and x = last).
            else
            {
                this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0,
                               currentPixelY, 0,
                               params.numCellsPerRowCol, params.numCellsPerRowCol,
                               CellType::FREE_CELL, false, true); // Ray trace obstacle from one pixel to another.
                this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0,
                               currentPixelY, params.numCellsPerRowCol - 1,
                               params.numCellsPerRowCol, params.numCellsPerRowCol,
                               CellType::FREE_CELL, false, true);
            }
        }
    }

    /**
     * @brief Perform map update.
     * 
     * This will generate brand new map from all sensor data.
     */
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
    }

protected:
    /**
     * @brief Get iterator to first element of internal vector that stores the map.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @return const_iterator.
     */
    virtual std::vector<int8_t>::const_iterator begin() const
    {
        return mapMatrix.begin();
    }

    /**
     * @brief Get iterator to end of internal vector that stores the map.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @return const_iterator.
     */
    virtual std::vector<int8_t>::const_iterator end() const override
    {
        return this->mapMsg.data.end();
    }

    /**
     * @brief Get element of internal vector that stores the map.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @param index Index.
     * @return Value of requested element.
     */
    virtual int8_t operator[](const unsigned int index) const override
    {
        return this->mapMsg.data[index];
    }

    /**
     * @brief Set sensor's map cell to given value.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @param y Row of map.
     * @param x Column of map.
     * @param val Value to set.
     */
    virtual void set(unsigned int y, unsigned int x, int8_t val) override
    {
        this->mapMsg.data[y * params.numCellsPerRowCol + x] = val;
    }

    /**
     * @brief Get value from given cell.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @param y Row of map.
     * @param x Column of map.
     * @return Value of cell.
     */
    virtual int8_t get(unsigned int y, unsigned int x) const override
    {
        return this->mapMsg.data[y * params.numCellsPerRowCol + x];
    }

    /**
     * @brief Reset map to CellType::UNKNOWN_CELL.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     */
    void reset() override
    {
        std::fill(this->mapMsg.data.begin(), this->mapMsg.data.end(), CellType::UNKNOWN_CELL);
    }

public:
    /**
     * @brief Class constructor. 
     * @param odomFrame Odometry frame.
     * @param mapFrame Map frame.
     * @param _mapUpdateRateMs How frequently update the map.
     * @param _maps Maps to fuse.
     */
    MapFuser(const std::string &odomFrame, const std::string &mapFrame,
             int _mapUpdateRateMs, const std::vector<std::reference_wrapper<AbstractMap>> _maps)
        : mapUpdateRateMs(_mapUpdateRateMs), maps(_maps)
    {
        this->mapMsg.info.resolution = params.cellSizeMeters;
        this->mapMsg.info.width = params.numCellsPerRowCol;
        this->mapMsg.info.height = params.numCellsPerRowCol;
        this->mapMsg.header.frame_id = mapFrame;

        //this->stampedTransform.header.frame_id = odomFrame;
        //this->stampedTransform.child_frame_id = mapFrame;
        this->stampedTransform.header.frame_id = mapFrame;
        this->stampedTransform.child_frame_id = odomFrame;

        this->resize();
    }

    /**
     * @brief Resizes map to accommodate map matrix after call to AbstractMap::setParams.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     */
    virtual void resize() override
    {
        this->mapMsg.data.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
        this->reset();
    }

    /**
     * @brief Callback fired on new odometry data.
     * 
     * This computes some stuff to accommodate movement between map updates.
     * 
     * @param odom Odometry message.
     */
    void onNewOdom(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // Convert to tf quaternion.
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

        // Convert to RPY angles.
        double temp1, temp2, theta;
        tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);

        // Get travelled distance since last odom msg.
        double travelledDistanceX = odom->pose.pose.position.x - this->latestOdomX;
        double travelledDistanceY = odom->pose.pose.position.y - this->latestOdomY;
        double travelledDistanceTheta = theta - this->latestOdomTheta;

        // Update odom offsets by travelled distance.
        this->odomOffsetX += travelledDistanceX;
        this->odomOffsetY += travelledDistanceY;
        this->odomOffsetTheta += travelledDistanceTheta;

        // Save stuff.
        this->latestOdomX = odom->pose.pose.position.x;
        this->latestOdomY = odom->pose.pose.position.y;
        this->latestOdomTheta = theta;
    }

    /**
     * @brief Try to update the map.
     * 
     * This will fail if not enough time has passed between map updates.
     * 
     * @return True is map updated.
     */
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

    /**
     * @brief Get ready to publish map message.
     * @return Map message.
     */
    const nav_msgs::OccupancyGrid &getMapMsg()
    {
        // Fill message with required data.
        this->mapMsg.header.stamp = ros::Time::now();
        this->mapMsg.info.origin.position.x = -params.mapSizeMeters / 2.0;
        this->mapMsg.info.origin.position.y = -params.mapSizeMeters / 2.0;

        return this->mapMsg;
    }

    /**
     * @brief Get ready to publish transformation.
     * @return Tf message.
     */
    const geometry_msgs::TransformStamped &getTransformMsg()
    {
        this->stampedTransform.header.stamp = ros::Time::now();

        // Accommodate odometry and it's offset.
        this->stampedTransform.transform.translation.x = this->latestOdomX - this->odomOffsetX;
        this->stampedTransform.transform.translation.y = this->latestOdomY - this->odomOffsetY;

        tf2::Quaternion q;
        q.setRPY(0, 0, this->latestOdomTheta - this->odomOffsetTheta);
        this->stampedTransform.transform.rotation.x = q.x();
        this->stampedTransform.transform.rotation.y = q.y();
        this->stampedTransform.transform.rotation.z = q.z();
        this->stampedTransform.transform.rotation.w = q.w();

        // The transform created above is valid for odom to map transform
        // but we need from map to odom.
        // So we need to invert it.
        tf::Transform tfTransform;
        tf::transformMsgToTF(this->stampedTransform.transform, tfTransform);
        tf::transformTFToMsg(tfTransform.inverse(), this->stampedTransform.transform);

        return this->stampedTransform;
    }
};