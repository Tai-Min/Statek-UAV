#include "../../include/voronoi_mapper/voronoi_map.hpp"

void VoronoiMap::occupancyGridToMat(const mapType &grid, cv::Mat &mat)
{
    mapType data(grid);

    for (int i = 0; i < data.size(); i++)
    {
        if (data[i] == 0)
            data[i] = 255;
        else
            data[i] = 0;
    }

    mat.create(params.numCellsPerRowCol, params.numCellsPerRowCol, CV_8UC1);
    std::copy(data.begin(), data.end(), mat.data);
}

void VoronoiMap::extractObstacleCorners(const cv::Mat &map, std::vector<cv::Point> &corners)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(map, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

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

double VoronoiMap::getDistanceToObstacle(unsigned int y, unsigned int x)
{
}

bool VoronoiMap::pointTooCloseToObstacle(unsigned int y, unsigned int x)
{
    // Ray trace free cells from center of the map to edges.
    for (int currentPixelY = 0; currentPixelY < params.numCellsPerRowCol; currentPixelY++)
    {
        // Check point to top and bottom edge of the map (y = first and every x) and (y = last and every x).
        if (currentPixelY == 0 || currentPixelY == params.numCellsPerRowCol - 1)
        {
            for (int currentPixelX = 0; currentPixelX < params.numCellsPerRowCol; currentPixelX++)
            {
            }
        }
        // Check point to left and right edge of the map (every other y and x = first) and (every other y and x = last).
        else
        {
        }
    }

    return false;
}

void VoronoiMap::filterVoronoiPoints(std::vector<cv::Point> &voronoi, const mapType &mapData)
{
    for (int i = 0; i < voronoi.size(); i++)
    {
        // Remove points out of range.
        while ((voronoi[i].x < 0 || voronoi[i].x >= params.numCellsPerRowCol || voronoi[i].y < 0 || voronoi[i].y >= params.numCellsPerRowCol) && voronoi.size() > i)
        {
            voronoi.erase(voronoi.begin() + i);
        }

        if (i >= voronoi.size())
            return;

        // Remove points not on free cells.
        if (get(mapData, voronoi[i].y, voronoi[i].x) != 0)
        {
            voronoi.erase(voronoi.begin() + i);
            i--;
            continue;
        }

        // Remove points too close to obstacles.
        if (pointTooCloseToObstacle(voronoi[i].y, voronoi[i].x))
        {
            voronoi.erase(voronoi.begin() + i);
            i--;
            continue;
        }
    }
}

int8_t VoronoiMap::get(const mapType &v, unsigned int y, unsigned int x)
{
    return v[y * params.numCellsPerRowCol + x];
}

void VoronoiMap::onNewLocalMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    cv::Mat mat;
    occupancyGridToMat(map->data, mat);

    std::vector<cv::Point> corners;
    extractObstacleCorners(mat, corners);

    std::vector<cv::Point> voronoi;
    getVoronoiPoints(corners, voronoi);

    filterVoronoiPoints(voronoi, map->data);

    // Insert robot's location.
    // We'll just assume that the robot is at the center of the map during update.
    voronoi.push_back(cv::Point(params.numCellsPerRowCol / 2, params.numCellsPerRowCol / 2));

    // Insert goal's location.
    voronoi.push_back(cv::Point(this->goalX, this->goalY));

    // TODO: Connect voronoi.

    // Display!
    cv::cvtColor(mat, mat, CV_GRAY2BGR);

    for (int i = 0; i < corners.size(); i++)
    {
        cv::circle(mat, corners[i], 1, cv::Scalar(0, 0, 255));
    }

    for (int i = 0; i < voronoi.size(); i++)
    {
        cv::circle(mat, voronoi[i], 1, cv::Scalar(255, 0, 0));
    }

    cv::namedWindow("T", 0);
    cv::resizeWindow("T", 800, 800);
    cv::resize(mat, mat, cv::Size(800, 800));
    cv::imshow("T", mat);
    cv::waitKey(33);
}

void VoronoiMap::setGoalPosition(double y, double x)
{
    double unused;
    this->transformPoint(x, y, unused);

    this->goalX = toIndex(x);
    this->goalY = toIndex(y);

    std::cout << this->goalX << ", " << this->goalY << std::endl;
}