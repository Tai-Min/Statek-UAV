#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Transform.h>
#include <statek_map/Graph.h>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "../abstract_map.hpp"
#include <limits>
#include <thread>
#include <mutex>

class VoronoiMap : public AbstractMap
{
private:
    void tfThreadFcn();
    std::atomic_bool stopTfThread = {false};
    std::atomic_bool robotTfReceived = {false};
    std::atomic_bool goalTfReceived = {false};
    std::thread tfThread;
    std::mutex robotTfMutex;
    std::mutex goalTfMutex;
    std::mutex goalLinkMutex;

    geometry_msgs::TransformStamped robotTransform;

    static double minimumGapSizeMeters;                        //!< Maximum allowed gap size between obstacles in meters.
    bool updatedSinceLastGet = false;                          //!< Flag to check whether map was updated since last map retreive using getGraph.
    statek_map::Graph voronoiGraph;                            //!< Stores voronoi graph ready to be published.
    double goalRawX = std::numeric_limits<double>::infinity(); //!< Goal X in some frame.
    double goalRawY = std::numeric_limits<double>::infinity(); //!< Goal Y in some frame.
    std::string goalLink = "";                                 //!< Tf link of the most recent goal message.
    const std::string localMapLink = "";                       //!< Tf link of local map.

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

    /**
     * @brief Insert anchor points (points around the edges of the map) whenever possible.
     */
    static int insertAnchors(std::vector<cv::Point> &voronoi, const mapType &grid, int num);

    /**
     * @brief Check whether point on given index is an anchor or not.
     * @param index Index of point to check.
     * @param numAnchors Number of anchors inserted to voronoi.
     * @param isGoal Whether the goal point was inserted to voronoi.
     * @return True if the point is an anchor.
     */
    static bool isAnchor(int index, const std::vector<cv::Point> &voronoi, int numAnchors, bool isGoal);

    /**
     * @brief Generate new statek_map::Graph message.
     * Called on new local map. Converts given voronoi diagram
     * into the message and also seeks possible neighbors and connects them.
     * @param voronoi Voronoi diagram to be converted to message.
     * @param grid Map data of local map.
     * @param numAnchors Number of anchors inserted to voronoi.
     */
    void generateMessage(const std::vector<cv::Point> &voronoi, const mapType &grid, int numAnchors);

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
     * @brief Class constructor.
     */
    VoronoiMap(const std::string &_localMapLink);

    ~VoronoiMap();

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
     * @return true if new graph was generated since last getGraphMsg.
     */
    bool newGraphAvailable();

    /**
     * @brief Get voronoi graph as ready to publish message.
     * @return Message with voronoi graph.
     */
    const statek_map::Graph &getGraphMsg();

    /**
     * @brief Resizes map to accommodate map matrix after call to AbstractMap::setParams.
     * This overrides default behavior from AbstractMap by changing some internal parameters.
     */
    virtual void resize() override;
};