#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "abstract_map.hpp"

class VoronoiMap : public AbstractMap
{
private:
    /**
     * @brief Convert ROS OccupancyGrid vector into OpenCV's Mat.
     * This function assumes that unknown cells are unavailable.
     * @param grid Grid to convert - this vector.
     * @param mat Binary output matrix where black means unavailable region and white free region.
     */
    static void occupancyGridToMat(const std::vector<int8_t> &grid, cv::Mat &mat)
    {
        std::vector<int8_t> data(grid);

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

    void extractCorners(const cv::Mat &map, std::vector<cv::Point> &corners)
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

    void getVoronoi(const std::vector<cv::Point> &corners, std::vector<cv::Point> &voronoi)
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

    void filterVoronoi(std::vector<cv::Point> &voronoi, const std::vector<int8_t> mapData)
    {
        for (int i = 0; i < voronoi.size(); i++)
        {
            // Remove all bad points.
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
            }
        }
    }

    static int8_t get(const std::vector<int8_t> &v, unsigned int y, unsigned int x)
    {
        return v[y * params.numCellsPerRowCol + x];
    }

public:
    void onNewData(const nav_msgs::OccupancyGrid::ConstPtr &map)
    {
        cv::Mat mat;
        occupancyGridToMat(map->data, mat);

        std::vector<cv::Point> corners;
        extractCorners(mat, corners);

        std::vector<cv::Point> voronoi;
        getVoronoi(corners, voronoi);

        filterVoronoi(voronoi, map->data);

        // Insert robot's location
        // The robot should be at the center of the map during update.
        voronoi.push_back(cv::Point(params.numCellsPerRowCol / 2, params.numCellsPerRowCol / 2));

        // TODO: Insert goal here to voronoi.

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
};