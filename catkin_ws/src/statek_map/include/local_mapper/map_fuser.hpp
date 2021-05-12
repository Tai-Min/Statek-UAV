#pragma once

#include <chrono>


#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include "../abstract_map.hpp"
#include "laser_scan_map.hpp"

class MapFuser : AbstractMap
{
private:
    // ROS stuff.
    nav_msgs::OccupancyGrid mapMsg;                   //!< Stores map and it's stuff.
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
    static bool isInRange(int y, int x, int sizeY, int sizeX);

    /**
     * @brief Perform sensor fusion.
     */
    void fuseMaps();

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
    bool rayTrace(int y0, int x0, int y1, int x1, int sizeX, int sizeY, int8_t cellType, bool bold = false, bool stopOnFilled = false);

    /**
     * @brief Check whether gap between given coordinates is considered small.
     * 
     * @param y0 Start y.
     * @param x0 Start x.
     * @param y1 End y.
     * @param x1 End x.
     * @return True if this gap is smaller than minimumGapSizeMeters from params.
     */
    bool isSmallGap(int y0, int x0, int y1, int x1) const;

    /**
     * @brief Close gaps that are smaller than minimumGapSizeMeters from params.
     * 
     * Should be called after map fusion.
     */
    void closeSmallGaps();

    /**
     * @brief Set all cells visible by the robot to CellType::FREE_CELL. 
     * 
     * Should be only called in updateMap() after all of the sensor data had been fused.
     */
    void rayTraceFreeCells();

    /**
     * @brief Perform map update.
     * 
     * This will generate brand new map from all sensor data.
     */
    void updateMap();

protected:
    /**
     * @brief Get iterator to first element of internal vector that stores the map.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @return const_iterator.
     */
    virtual std::vector<int8_t>::const_iterator begin() const override;

    /**
     * @brief Get iterator to end of internal vector that stores the map.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @return const_iterator.
     */
    virtual std::vector<int8_t>::const_iterator end() const override;

    /**
     * @brief Get element of internal vector that stores the map.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @param index Index.
     * @return Value of requested element.
     */
    virtual int8_t operator[](const unsigned int index) const override;

    /**
     * @brief Set sensor's map cell to given value.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @param y Row of map.
     * @param x Column of map.
     * @param val Value to set.
     */
    virtual void set(unsigned int y, unsigned int x, int8_t val) override;

    /**
     * @brief Get value from given cell.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     * 
     * @param y Row of map.
     * @param x Column of map.
     * @return Value of cell.
     */
    virtual int8_t get(unsigned int y, unsigned int x) const override;

    /**
     * @brief Reset map to CellType::UNKNOWN_CELL.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     */
    void reset() override;

public:
    /**
     * @brief Class constructor. 
     * @param odomFrame Odometry frame.
     * @param mapFrame Map frame.
     * @param _mapUpdateRateMs How frequently update the map.
     * @param _maps Maps to fuse.
     */
    MapFuser(const std::string &odomFrame,
             const std::string &mapFrame,
             int _mapUpdateRateMs,
             const std::vector<std::reference_wrapper<AbstractMap>> _maps);

    /**
     * @brief Resizes map to accommodate map matrix after call to AbstractMap::setParams.
     * 
     * This overrides default behavior from AbstractMap by replacing mapMatrix with mapMsg.data.
     */
    virtual void resize() override;

    /**
     * @brief Callback fired on new odometry data.
     * 
     * This computes some stuff to accommodate movement between map updates.
     * 
     * @param odom Odometry message.
     */
    void onNewOdom(const nav_msgs::Odometry::ConstPtr &odom);

    /**
     * @brief Try to update the map.
     * 
     * This will fail if not enough time has passed between map updates.
     * 
     * @return True is map updated.
     */
    bool tryUpdateMap();

    /**
     * @brief Get ready to publish map message.
     * @return Map message.
     */
    const nav_msgs::OccupancyGrid &getMapMsg();

    /**
     * @brief Get ready to publish transformation.
     * @return Tf message.
     */
    const geometry_msgs::TransformStamped &getTransformMsg();
};