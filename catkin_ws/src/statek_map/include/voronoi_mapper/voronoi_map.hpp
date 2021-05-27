#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <statek_map/Graph.h>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "../abstract_map.hpp"

class VoronoiMap : public AbstractMap
{
private:
    static double minimumGapSizeMeters;
    bool updatedSinceLastGet = false; //!< Flag to check whether map was updated since last map retreive using getGraph.
    statek_map::Graph voronoiGraph;   //!< Stores voronoi graph ready to be published.
    double goalRawX = 0;              //!< Goal X in earth frame.
    double goalRawY = 0;              //!< Goal Y in earth frame.
    int goalX = 0;                    //!< X index position of goal from center of local map.
    int goalY = 0;                    //!< Y index position of goal from center of local map.

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
     * @brief Check whether given point is too close to obstacle.
     * Too close means that it's closer than sqrt of minimumGapSizeMetersSquared from AbstractMap::MapParams.
     * @param y Row index of the point.
     * @param x Column index of the point.
     * @param grid Grid map from onNewLocalMap method.
     * @return True if point is too close to obstacle.
     */
    static bool pointTooCloseToObstacle(unsigned int y, unsigned int x, const mapType &grid);

    /**
     * @brief Check whether travel from point 0 to 1 is possible.
     * @param y0 Y0.
     * @param x0 X0.
     * @param y1 Y1.
     * @param x1 X1.
     * @param grid Grid map from onNewLocalMap method.
     * @param checkSurroundings Check if robot can move through ray traced path.
     * @return less than 0 If ray trace successfull or distance in pixels from x0 to pixel where algorithm failed.
     * i.e this pixel could be an obstacle or could be too close to obstacle.
     */
    static double rayTrace(int y0, int x0, int y1, int x1, const mapType &grid, int increment = 1, bool checkSurroundings = true);

    /**
     * @brief Filter voronoi points so only these accessible by robot remain.
     * @param voronoi Vector of voronoi points.
     * @param mapData Data from OccupancyGrid message.
     */
    static void filterVoronoiPoints(std::vector<cv::Point> &voronoi, const mapType &mapData);

    void generateMessage(const std::vector<cv::Point> &voronoi, const mapType &grid);

    /**
     * @brief Get element from some vector while treating it as matrix.
     * Width and height of this matrix is set using AbstractMap::setParams.
     * @param v Vector to get value from.
     * @param y Row.
     * @param x Column.
     */
    static int8_t get(const mapType &v, unsigned int y, unsigned int x);

public:
    VoronoiMap();

    /**
     * @brief Callback called on update of local grid map.
     * Creates voronoi graph based on this map.
     * @param map Updated local map.
     */
    void onNewLocalMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

    /**
     * @brief Callback with new short term goal in earth frame.
     * @param goal New short term goal.
     */
    void onNewShortTermGoal(const geometry_msgs::PoseStamped &goal);

    /**
     * @brief Check whether graph updated.
     */
    bool newGraphAvailable();

    const statek_map::Graph &getGraph();

    /**
     * @brief Save transform received from tf transform broadcaster.
     * Overrides default behavior from AbstractMap by updating goal position in reference to local map.
     * 
     * @param _transform Desired transform.
     */
    virtual void setTransform(const geometry_msgs::TransformStamped &_transform) override;

    /**
     * @brief Resizes map to accommodate map matrix after call to AbstractMap::setParams.
     * This overrides default behavior from AbstractMap by changing some internal parameters.
     */
    virtual void resize() override;
};