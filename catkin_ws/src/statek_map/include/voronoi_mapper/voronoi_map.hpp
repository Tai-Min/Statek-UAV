#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <opencv2/core/mat.hpp>
#include "../abstract_map.hpp"

class VoronoiMap : public AbstractMap
{
private:
    double minimumGapSizeMeters;

    int goalX; //!< X index position of goal from center of local map.
    int goalY; //!< Y index position of goal from center of local map.

    /**
     * @brief Convert ROS OccupancyGrid vector into OpenCV's Mat.
     * This function assumes that unknown cells are unavailable.
     * @param grid Grid to convert - this vector.
     * @param mat Binary output matrix where black means unavailable region and white free region.
     */
    static void occupancyGridToMat(const mapType &grid, cv::Mat &mat);

    /**
     * @brief Extract obstacle corners from given binary map.
     * @param map Input map.
     * @param conterns Extracted obstacle corners.
     */
    static void extractObstacleCorners(const cv::Mat &map, std::vector<cv::Point> &corners);

    /**
     * @brief Find voronoi points from corners of the obstacles.
     * @param corners Conters of the obstacles.
     * @param voronoi Found points.
     */
    static void getVoronoiPoints(const std::vector<cv::Point> &corners, std::vector<cv::Point> &voronoi);

    /**
     * @brief Get distance to closest obstacle from given point.
     * Result is in meters.
     * @param y Row index of the point.
     * @param x Column index of the point.
     */
    static double getDistanceToObstacle(unsigned int y, unsigned int x);

    /**
     * @brief Check whether given point is too close to obstacle.
     * Too close means that it's closer than sqrt of minimumGapSizeMetersSquared from AbstractMap::MapParams.
     * @param y Row index of the point.
     * @param x Column index of the point.
     */
    static bool pointTooCloseToObstacle(unsigned int y, unsigned int x);

    /**
     * @brief Filter voronoi points so only these accessible by robot remain.
     * @param voronoi Vector of voronoi points.
     * @param mapData Data from OccupancyGrid message.
     */
    static void filterVoronoiPoints(std::vector<cv::Point> &voronoi, const mapType &mapData);

    /**
     * @brief Get element from some vector while treating it as matrix.
     * Width and height of this matrix is set using AbstractMap::setParams.
     * @param v Vector to get value from.
     * @param y Row.
     * @param x Column.
     */
    static int8_t get(const mapType &v, unsigned int y, unsigned int x);

public:
    /**
     * @brief Callback called on update of local grid map.
     * Creates voronoi graph based on this map.
     * @param map Updated local map.
     */
    void onNewLocalMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

    /**
     * @brief Set position of the goal.
     * @param x X position in meters from the center of earth's tangent plane (from earth link).
     * @param y Y position in meters from the center of earth's tangent plane (from earth link).
     */
    void setGoalPosition(double y, double x);
};