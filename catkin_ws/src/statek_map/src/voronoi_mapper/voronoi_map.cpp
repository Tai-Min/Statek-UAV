#include "../../include/voronoi_mapper/voronoi_map.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tf/transform_broadcaster.h>
#include "../../include/common.hpp"
#include <chrono>

double VoronoiMap::minimumGapSizeMeters = 0;

VoronoiMap::VoronoiMap(const std::string &_localMapLink)
    : localMapLink(_localMapLink), tfThread(&VoronoiMap::tfThreadFcn, this)
{
    this->minimumGapSizeMeters = sqrt(params.minimumGapSizeMetersSquared);
}

VoronoiMap::~VoronoiMap()
{
    stopTfThread = true;
    tfThread.join();
}

void VoronoiMap::tfThreadFcn()
{
    auto previousRobotTfTime = std::chrono::steady_clock::now();
    auto previousGoalTfTime = std::chrono::steady_clock::now();

    while (!stopTfThread)
    {
        auto now = std::chrono::steady_clock::now();

        // Check for tf timeouts to stop the mapper.
        auto robotTfElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - previousRobotTfTime).count();
        if (robotTfElapsed > 3000)
            robotTfReceived = false;

        auto goalTfElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - previousGoalTfTime).count();
        if (goalTfElapsed > 3000)
            goalTfReceived = false;

        // Get location of the robot on local map.
        bool ok = false;
        {
            std::lock_guard<std::mutex> robotLck(robotTfMutex);
            robotTransform = getTransform("statek/map/local_map_link", "statek/base_footprint", ok);
        }

        if (ok)
        {
            robotTfReceived = true;
            previousRobotTfTime = std::chrono::steady_clock::now();
        }

        // Get most recent transform of the goal.
        ok = false;
        {
            std::lock_guard<std::mutex> goalLck(goalTfMutex);
            std::lock_guard<std::mutex> goalLinkLck(goalLinkMutex);
            geometry_msgs::TransformStamped goalTransform = getTransform(this->localMapLink, this->goalLink, ok);
            AbstractMap::setTransform(goalTransform);
        }

        if (ok)
        {
            goalTfReceived = true;
            previousGoalTfTime = std::chrono::steady_clock::now();
        }
    }
    
}

void VoronoiMap::occupancyGridToMat(const mapType &grid, cv::Mat &mat)
{
    mapType data(grid);

    for (int i = 0; i < data.size(); i++)
    {
        if (data[i] == CellType::FREE_CELL || data[i] == CellType::UNKNOWN_CELL)
            data[i] = 0;
        else
            data[i] = 255;
    }

    mat.create(params.numCellsPerRowCol, params.numCellsPerRowCol, CV_8UC1);
    std::copy(data.begin(), data.end(), mat.data);
}

void VoronoiMap::extractObstacleCorners(const cv::Mat &map, std::vector<cv::Point> &corners)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(map, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contoursApprox;
    contoursApprox.resize(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(contours[i]), contoursApprox[i], 1.5, true);
    }

    int cntr = 0;
    for (int i = 0; i < contoursApprox.size(); i++)
    {
        for (int j = 0; j < contoursApprox[i].size(); j++)
        {
            if (cntr >= corners.size())
                corners.push_back(contoursApprox[i][j]);
            else
                corners[cntr] = contoursApprox[i][j];
            cntr++;
        }
    }
}

void VoronoiMap::getVoronoiPoints(const std::vector<cv::Point> &corners, std::vector<cv::Point> &voronoi)
{
    cv::Rect rect(0, 0, params.numCellsPerRowCol, params.numCellsPerRowCol);
    cv::Subdiv2D subdiv(rect);

    for (int i = 0; i < corners.size(); i++)
    {
        // Ignore frame
        if (corners[i] != cv::Point(0, 0) &&
            corners[i] != cv::Point(params.numCellsPerRowCol - 1, 0) &&
            corners[i] != cv::Point(0, params.numCellsPerRowCol - 1) &&
            corners[i] != cv::Point(params.numCellsPerRowCol - 1, params.numCellsPerRowCol - 1))
            subdiv.insert(corners[i]);
    }

    std::vector<std::vector<cv::Point2f>> facets;
    std::vector<cv::Point2f> centers;
    subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

    int cntr = 0;
    for (size_t i = 0; i < facets.size(); i++)
    {
        for (size_t j = 0; j < facets[i].size(); j++)
        {
            if (cntr >= voronoi.size())
                voronoi.push_back(facets[i][j]);
            else
                voronoi[cntr] = facets[i][j];
            cntr++;
        }
    }
}

bool VoronoiMap::pointTooCloseToObstacle(unsigned int y, unsigned int x, const mapType &grid)
{
    // Increment ray tracing around edges by more pixels for performance.
    // This will add some error but w/e for bigger maps.
    static const int edgeIncrement = params.numCellsPerRowCol / 10;

    // How far should raytracing algorithm jump in pixels
    // when checking if track between points is safe.
    // 1 means check every pixel, 2 check every second pixel etc.
    // This also will add some error but w/e for bigger maps.
    static const int rayIncrement = 3;

    // Ray trace free cells from given point to edges of the map.
    for (int currentPixelY = 0; currentPixelY < params.numCellsPerRowCol; currentPixelY += edgeIncrement)
    {
        // Check point to top and bottom edge of the map (y = first and every x) and (y = last and every x).
        if (currentPixelY == 0 || currentPixelY == params.numCellsPerRowCol - 1)
        {
            for (int currentPixelX = 0; currentPixelX < params.numCellsPerRowCol; currentPixelX += edgeIncrement)
            {
                double distance;
                distance = rayTrace(y, x, 0, currentPixelX, grid, rayIncrement, false);
                if (distance >= 0 && distance * params.cellSizeMeters < minimumGapSizeMeters * 0.5)
                    return true;

                distance = rayTrace(y, x, params.numCellsPerRowCol - 1, currentPixelX, grid, rayIncrement, false);
                if (distance >= 0 && distance * params.cellSizeMeters < minimumGapSizeMeters * 0.5)
                    return true;
            }
        }
        // Check point to left and right edge of the map (every other y and x = first) and (every other y and x = last).
        else
        {
            double distance;
            distance = rayTrace(y, x, currentPixelY, 0, grid, rayIncrement, false);
            if (distance >= 0 && distance * params.cellSizeMeters < minimumGapSizeMeters * 0.5)
                return true;

            distance = rayTrace(y, x, currentPixelY, params.numCellsPerRowCol - 1, grid, rayIncrement, false);
            if (distance >= 0 && distance * params.cellSizeMeters < minimumGapSizeMeters * 0.5)
                return true;
        }
    }
    return false;
}

double VoronoiMap::rayTrace(int y0, int x0, int y1, int x1, const mapType &grid, int increment, bool checkSurroundings)
{
    int d, dx, dy, ai, bi, xi, yi;
    int x = x0, y = y0;
    if (x0 < x1)
    {
        xi = increment;
        dx = x1 - x0;
    }
    else
    {
        xi = -increment;
        dx = x0 - x1;
    }
    if (y0 < y1)
    {
        yi = increment;
        dy = y1 - y0;
    }
    else
    {
        yi = -increment;
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

            // Algorithm is on the edge, so it's probably possible to move further.
            if (!isValidPoint(y, x))
                return -1;

            // Stop on obstacle.
            if (get(grid, y, x) != CellType::FREE_CELL && get(grid, y, x) != CellType::UNKNOWN_CELL)
                return sqrt(pow(x0 - x, 2) + pow(y0 - y, 2));

            // Stop if there is not enough space around this point.
            if (checkSurroundings)
            {
                if (pointTooCloseToObstacle(y, x, grid))
                    return sqrt(pow(x0 - x, 2) + pow(y0 - y, 2));
            }
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

            // Algorithm is on the edge, so it's probably possible to move further.
            if (!isValidPoint(y, x))
                return -1;

            // Stop on obstacle.
            if (get(grid, y, x) != CellType::FREE_CELL && get(grid, y, x) != CellType::UNKNOWN_CELL)
                return sqrt(pow(x0 - x, 2) + pow(y0 - y, 2));

            // Stop if there is not enough space around this point.
            if (checkSurroundings)
            {
                if (pointTooCloseToObstacle(y, x, grid))
                    return sqrt(pow(x0 - x, 2) + pow(y0 - y, 2));
            }
        }
    }
    return -1;
}

void VoronoiMap::filterVoronoiPoints(std::vector<cv::Point> &voronoi, const mapType &mapData)
{
    std::vector<cv::Point> voronoiResult;
    std::copy_if(voronoi.begin(), voronoi.end(), std::back_inserter(voronoiResult), [mapData](cv::Point p)
                 {
                     bool isOk = p.y < params.numCellsPerRowCol && p.y >= 0 && p.x < params.numCellsPerRowCol && p.x >= 0 && get(mapData, p.y, p.x) == 0 && !pointTooCloseToObstacle(p.y, p.x, mapData);
                     return isOk;
                 });
    voronoi = std::move(voronoiResult);
}

int VoronoiMap::insertAnchors(std::vector<cv::Point> &voronoi, const mapType &grid, int num)
{
    int inserted = 0;
    num = num - 1;

    int startPosition = 0;
    int endPosition = params.numCellsPerRowCol - 1;

    for (int i = 0; i <= num; i++)
    {
        double position = (double)endPosition * i / (double)num;

        if (get(grid, position, startPosition) == CellType::FREE_CELL)
        {
            voronoi.push_back(cv::Point(startPosition, position));
            inserted++;
        }

        if (position != startPosition && get(grid, startPosition, position) == CellType::FREE_CELL)
        {
            voronoi.push_back(cv::Point(position, startPosition));
            inserted++;
        }

        if (get(grid, position, endPosition) == CellType::FREE_CELL)
        {
            voronoi.push_back(cv::Point(endPosition, position));
            inserted++;
        }

        if (position != endPosition && get(grid, endPosition, position) == CellType::FREE_CELL)
        {
            voronoi.push_back(cv::Point(position, endPosition));
            inserted++;
        }
    }

    return inserted;
}

bool VoronoiMap::isAnchor(int index, const std::vector<cv::Point> &voronoi, int numAnchors, bool isGoal)
{
    int startGoalNum = 1; // Count only start.
    if (isGoal)
        startGoalNum = 2; // Count start and goal.

    int anchorsBegin = voronoi.size() - numAnchors - startGoalNum; // Points to first anchor.
    int anchorsEnd = voronoi.size() - startGoalNum;                // Points index after last anchor.

    if (index >= anchorsBegin && index < anchorsEnd)
        return true;

    return false;
}

void VoronoiMap::generateMessage(const std::vector<cv::Point> &voronoi, const mapType &grid, int numAnchors)
{
    this->voronoiGraph.nodes.clear();
    for (int i = 0; i < voronoi.size(); i++)
    {
        statek_map::GraphNode node;
        node.id = i;
        node.point.x = this->toMeters(voronoi[i].x);
        node.point.y = this->toMeters(voronoi[i].y);

        this->voronoiGraph.nodes.push_back(node);
    }

    bool isGoal = false;

    // Last point is a goal if there is one.
    // Second to last point is a start.
    // Point inf should be unreachable.
    if (std::isfinite(this->goalRawX) && std::isfinite(this->goalRawY))
    {
        this->voronoiGraph.nodes[voronoi.size() - 1].isGoal = true;
        this->voronoiGraph.nodes[voronoi.size() - 2].isStart = true;
        isGoal = true;
    }
    else
    {
        this->voronoiGraph.nodes[voronoi.size() - 1].isStart = true; // If there is no goal then last point is a start.
    }

    // For every pushed node...
    for (int i = 0; i < voronoi.size(); i++)
    {
        // Ignore points outside grid map
        // Such as goal as for those points rayTrace
        // would always return negative value.
        if (!isValidPoint(voronoi[i].y, voronoi[i].x))
        {
            continue;
        }

        // Check whether movement between two points is possible
        // and if so then add them to neighbors of each other.
        // Also ignore connections from anchor to anchor for performance reasons.
        for (int j = i + 1; j < voronoi.size(); j++)
        {
            if (isAnchor(i, voronoi, numAnchors, isGoal) &&
                isAnchor(j, voronoi, numAnchors, isGoal))
            {
                continue;
            }

            if (rayTrace(voronoi[i].y, voronoi[i].x,
                         voronoi[j].y, voronoi[j].x,
                         grid) < 0)
            {
                this->voronoiGraph.nodes[i].neighbors.push_back(j);
                this->voronoiGraph.nodes[j].neighbors.push_back(i);
            }
        }
    }
}

void VoronoiMap::onNewLocalMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    this->voronoiGraph.header.frame_id = map->header.frame_id;

    cv::Mat mat;
    occupancyGridToMat(map->data, mat);

    std::vector<cv::Point> corners;
    extractObstacleCorners(mat, corners);

    std::vector<cv::Point> voronoi;
    getVoronoiPoints(corners, voronoi);

    filterVoronoiPoints(voronoi, map->data);

    // Insert some anchor points.
    double gapSizeCells = this->minimumGapSizeMeters / params.cellSizeMeters;
    int numAnchors = params.numCellsPerRowCol / gapSizeCells;
    numAnchors = insertAnchors(voronoi, map->data, numAnchors);

    // Add offsets from map link to base footprint.
    double posX = 0;
    double posY = 0;
    if (robotTfReceived)
    {
        std::lock_guard<std::mutex> robotLck(robotTfMutex);
        posX = robotTransform.transform.translation.x;
        posY = robotTransform.transform.translation.y;
    }

    // Insert robot's location.
    voronoi.push_back(cv::Point(params.numCellsPerRowCol / 2 + posX, params.numCellsPerRowCol / 2 + posY));

    // Insert goal's location.
    // inf in X or Y is treated as no goal.
    if (std::isfinite(this->goalRawX) && std::isfinite(this->goalRawY) && goalTfReceived)
    {
        // Transform goal from earth to local map.
        double tempX = this->goalRawX;
        double tempY = this->goalRawY;
        double unused = 0;

        {
            std::lock_guard<std::mutex> goalLck(goalTfMutex);
            this->transformPoint(tempX, tempY, unused);
        }

        // Insert points to voronoi.
        voronoi.push_back(cv::Point(toIndex(tempX), toIndex(tempY)));
    }

    // Add nodes to message.
    generateMessage(voronoi, map->data, numAnchors);

    this->updatedSinceLastGet = true;
}

void VoronoiMap::onNewShortTermGoal(const geometry_msgs::PoseStamped &goal)
{
    {
        std::lock_guard<std::mutex> goalLinkLck(goalLinkMutex);
        this->goalLink = goal.header.frame_id;
    }

    this->goalRawX = goal.pose.position.x;
    this->goalRawY = goal.pose.position.y;
}

bool VoronoiMap::newGraphAvailable()
{
    return this->updatedSinceLastGet;
}

const statek_map::Graph &VoronoiMap::getGraphMsg()
{
    this->updatedSinceLastGet = false;

    this->voronoiGraph.header.stamp = ros::Time::now();
    return this->voronoiGraph;
}

void VoronoiMap::resize()
{
    AbstractMap::resize();
    this->minimumGapSizeMeters = sqrt(params.minimumGapSizeMetersSquared);
}