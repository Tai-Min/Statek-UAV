#include "../include/abstract_map.hpp"
#include <tf2/transform_datatypes.h>

AbstractMap::MapParams AbstractMap::params = {0, 0, 0, 0};

AbstractMap::AbstractMap()
{
    this->resize();
}

bool AbstractMap::rayTrace(int y0, int x0, int y1, int x1, int8_t cellType, bool bold, bool stopOnFilled)
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
    if (cellType == FREE_CELL)
    {
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
                if (isValidPoint(boldY, boldX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                    this->set(boldY, boldX, cellType);

                boldX = x - xi;
                boldY = y;
                if (isValidPoint(boldY, boldX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                    this->set(boldY, boldX, cellType);
            }

            // Stop on obstacle.
            if (this->get(y, x) == CellType::OBSTACLE_CELL)
                return false;

            // Stop on filled gap if requested.
            if (this->get(y, x) == CellType::FILLED_GAP && stopOnFilled)
                return false;

            if (isValidPoint(y, x))
                this->set(y, x, cellType);
            else
                return false;
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
                if (isValidPoint(boldY, boldX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                    this->set(boldY, boldX, cellType);

                boldX = x - xi;
                boldY = y;
                if (isValidPoint(boldY, boldX) && this->get(boldY, boldX) == CellType::UNKNOWN_CELL)
                    this->set(boldY, boldX, cellType);
            }

            // Stop on obstacle.
            if (this->get(y, x) == CellType::OBSTACLE_CELL)
                return false;

            // Stop on filled gap if requested.
            if (this->get(y, x) == CellType::FILLED_GAP && stopOnFilled)
                return false;

            if (isValidPoint(y, x))
                this->set(y, x, cellType);
            else
                return false;
        }
    }
    return true;
}

bool AbstractMap::isValidPoint(int y, int x)
{
    if (y < 0 || y >= params.numCellsPerRowCol)
        return false;

    if (x < 0 || x >= params.numCellsPerRowCol)
        return false;

    return true;
}

void AbstractMap::transformPoint(double &x, double &y, double &z)
{
    double tx = x, ty = y, tz = z;
    tf2::Vector3 point(tx, ty, tz);

    point = this->transform * point;
    x = point.getX();
    y = point.getY();
    z = point.getZ();
}

AbstractMap::mapType::const_iterator AbstractMap::begin() const
{
    return mapMatrix.begin();
}

AbstractMap::mapType::const_iterator AbstractMap::end() const
{
    return mapMatrix.end();
}

AbstractMap::mapType::size_type AbstractMap::size() const
{
    return mapMatrix.size();
}

int8_t AbstractMap::operator[](const unsigned int index) const
{
    return mapMatrix[index];
}

void AbstractMap::set(mapType &map, unsigned int y, unsigned int x, int8_t val)
{
    map[y * params.numCellsPerRowCol + x] = val;
}

int8_t AbstractMap::get(const mapType &map, unsigned int y, unsigned int x)
{
    return map[y * params.numCellsPerRowCol + x];
}

void AbstractMap::set(unsigned int y, unsigned int x, int8_t val)
{
    mapMatrix[y * params.numCellsPerRowCol + x] = val;
}

int8_t AbstractMap::get(unsigned int y, unsigned int x) const
{
    return mapMatrix[y * params.numCellsPerRowCol + x];
}

void AbstractMap::reset()
{
    std::fill(this->mapMatrix.begin(), this->mapMatrix.end(), CellType::UNKNOWN_CELL);
}

void AbstractMap::setParams(const MapParams &_params)
{
    params = _params;
}

int AbstractMap::toIndex(double val)
{
    val /= params.cellSizeMeters;
    val += params.numCellsPerRowCol / 2.0;
    return val;
}

double AbstractMap::toMeters(int idx)
{
    double result = idx;
    result -= params.numCellsPerRowCol / 2.0;
    return result * params.cellSizeMeters;
}

void AbstractMap::setTransform(const geometry_msgs::TransformStamped &_transform)
{
    tf2::convert(_transform.transform, this->transform);
}

void AbstractMap::resize()
{
    mapMatrix.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
    this->reset();
}