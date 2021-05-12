#include "../include/abstract_map.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

AbstractMap::MapParams AbstractMap::params = {0, 0, 0, 0};

AbstractMap::AbstractMap()
{
    this->resize();
}

void AbstractMap::updateRotationMultipliers()
{
    tf2::Quaternion quatTf;
    tf2::fromMsg(this->transform.transform.rotation, quatTf);
    quatTf.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(quatTf).getRPY(roll, pitch, yaw);

    // https://en.wikipedia.org/wiki/Rotation_matrix
    this->xx = cos(yaw) * cos(pitch);
    this->xy = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    this->xz = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);

    this->yx = sin(yaw) * cos(pitch);
    this->yy = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    this->yz = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

    this->zx = -sin(pitch);
    this->zy = cos(pitch) * sin(roll);
    this->zz = cos(pitch) * cos(roll);
}

void AbstractMap::translatePoint(double &x, double &y, double &z)
{
    x += this->transform.transform.translation.x;
    y += this->transform.transform.translation.y;
    z += this->transform.transform.translation.z;
}

void AbstractMap::rotatePoint(double &x, double &y, double &z)
{
    double xTemp = x;
    double yTemp = y;
    double zTemp = z;

    x = xTemp * this->xx + yTemp * this->xy + zTemp * this->xz;
    y = xTemp * this->yx + yTemp * this->yy + zTemp * this->yz;
    z = xTemp * this->zx + yTemp * this->zy + zTemp * this->zz;
}

void AbstractMap::transformPoint(double &x, double &y, double &z)
{
    this->translatePoint(x, y, z);
    this->rotatePoint(x, y, z);
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

double AbstractMap::toMeters(unsigned int idx)
{
    double result = idx;
    result -= params.numCellsPerRowCol / 2.0;
    return result * params.cellSizeMeters;
}

void AbstractMap::setTransform(const geometry_msgs::TransformStamped &_transform)
{
    this->transform = _transform;

    this->updateRotationMultipliers();
}

void AbstractMap::resize()
{
    mapMatrix.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
    this->reset();
}