#include "../include/abstract_map.hpp"
#include <tf2/transform_datatypes.h>

AbstractMap::MapParams AbstractMap::params = {0, 0, 0, 0};

AbstractMap::AbstractMap()
{
    this->resize();
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

int8_t AbstractMap::operator[](const unsigned int index) const
{
    return mapMatrix[index];
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