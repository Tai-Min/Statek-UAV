#include "../../include/local_mapper/map_fuser.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

MapFuser::MapFuser(const std::string &odomFrame, const std::string &mapFrame,
                   int _mapUpdateRateMs, const std::vector<std::reference_wrapper<AbstractMap>> _maps)
    : mapUpdateRateMs(_mapUpdateRateMs), maps(_maps)
{
    this->mapMsg.info.resolution = params.cellSizeMeters;
    this->mapMsg.info.width = params.numCellsPerRowCol;
    this->mapMsg.info.height = params.numCellsPerRowCol;
    this->mapMsg.header.frame_id = mapFrame;

    this->transformMsg.header.frame_id = mapFrame;
    this->transformMsg.child_frame_id = odomFrame;

    this->resize();
}

void MapFuser::fuseMaps()
{
    if (!maps.size())
        return;

    for (int i = 0; i < maps[0].get().size(); i++)
    {
        mapMsg.data[i] = maps[0].get()[i];
        for (int j = 1; j < maps.size(); j++)
            mapMsg.data[i] = (maps[j].get()[i] > 0) ? CellType::OBSTACLE_CELL : mapMsg.data[i];
    }
}

bool MapFuser::isSmallGap(int y0, int x0, int y1, int x1) const
{
    double diffXMeters = toMeters(x1) - toMeters(x0);
    double diffXSquaredMeters = diffXMeters * diffXMeters;

    double diffYMeters = toMeters(y1) - toMeters(y0);
    double diffYSquaredMeters = diffYMeters * diffYMeters;

    return (diffXSquaredMeters + diffYSquaredMeters) < params.minimumGapSizeMetersSquared;
}

void MapFuser::closeSmallGaps()
{
    for (int currentPixelY = 0; currentPixelY < params.numCellsPerRowCol; currentPixelY++)
    {
        for (int currentPixelX = 0; currentPixelX < params.numCellsPerRowCol; currentPixelX++)
        {
            // Not an obstacle so nothing to check.
            if (this->get(currentPixelY, currentPixelX) != CellType::OBSTACLE_CELL)
                continue;

            for (int checkedPixelY = currentPixelY; checkedPixelY < params.numCellsPerRowCol; checkedPixelY++)
            {
                for (int checkedPixelX = 0; checkedPixelX < params.numCellsPerRowCol; checkedPixelX++)
                {
                    // Not an obstacle so nothing to check.
                    if (this->get(checkedPixelY, checkedPixelX) != CellType::OBSTACLE_CELL)
                        continue;

                    // Ray trace filled gap from one pixel to another.
                    if (this->isSmallGap(currentPixelY, currentPixelX, checkedPixelY, checkedPixelX))
                        this->rayTrace(currentPixelY, currentPixelX,
                                       checkedPixelY, checkedPixelX,
                                       CellType::FILLED_GAP, true, false);
                }
            }
        }
    }
}

void MapFuser::rayTraceFreeCells()
{
    // Ray trace free cells from center of the map to edges.
    for (int currentPixelY = 0; currentPixelY < params.numCellsPerRowCol; currentPixelY++)
    {
        // Fill (y = first and every x) and (y = last and every x).
        if (currentPixelY == 0 || currentPixelY == params.numCellsPerRowCol - 1)
        {
            for (int currentPixelX = 0; currentPixelX < params.numCellsPerRowCol; currentPixelX++)
            {

                this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0,
                               currentPixelY, currentPixelX,
                               CellType::FREE_CELL, false, true); // Ray trace obstacle from one pixel to another.
            }
        }
        // fill (every other y and x = first) and (every other y and x = last).
        else
        {
            this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0,
                           currentPixelY, 0,
                           CellType::FREE_CELL, false, true); // Ray trace obstacle from one pixel to another.
            this->rayTrace(params.numCellsPerRowCol / 2.0, params.numCellsPerRowCol / 2.0,
                           currentPixelY, params.numCellsPerRowCol - 1,
                           CellType::FREE_CELL, false, true);
        }
    }
}

void MapFuser::updateMap()
{
    this->reset();
    this->fuseMaps();
    this->closeSmallGaps();
    this->rayTraceFreeCells();

    // Map generated so reset offset.
    this->odomOffsetX = 0;
    this->odomOffsetY = 0;
    this->odomOffsetTheta = 0;
}

AbstractMap::mapType::const_iterator MapFuser::begin() const
{
    return mapMatrix.begin();
}

AbstractMap::mapType::const_iterator MapFuser::end() const
{
    return this->mapMsg.data.end();
}

int8_t MapFuser::operator[](const unsigned int index) const
{
    return this->mapMsg.data[index];
}

void MapFuser::set(unsigned int y, unsigned int x, int8_t val)
{
    this->mapMsg.data[y * params.numCellsPerRowCol + x] = val;
}

int8_t MapFuser::get(unsigned int y, unsigned int x) const
{
    return this->mapMsg.data[y * params.numCellsPerRowCol + x];
}

void MapFuser::reset()
{
    std::fill(this->mapMsg.data.begin(), this->mapMsg.data.end(), CellType::UNKNOWN_CELL);
}

void MapFuser::resize()
{
    this->mapMsg.data.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
    this->reset();
}

void MapFuser::onNewOdom(const nav_msgs::Odometry::ConstPtr &odom)
{
    // Convert to tf quaternion.
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

    // Convert to RPY angles.
    double temp1, temp2, theta;
    tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);

    // Get travelled distance since last odom msg.
    double travelledDistanceX = odom->pose.pose.position.x - this->latestOdomX;
    double travelledDistanceY = odom->pose.pose.position.y - this->latestOdomY;
    double travelledDistanceTheta = theta - this->latestOdomTheta;

    // Update odom offsets by travelled distance.
    this->odomOffsetX += travelledDistanceX;
    this->odomOffsetY += travelledDistanceY;
    this->odomOffsetTheta += travelledDistanceTheta;

    // Save stuff.
    this->latestOdomX = odom->pose.pose.position.x;
    this->latestOdomY = odom->pose.pose.position.y;
    this->latestOdomTheta = theta;
}

bool MapFuser::tryUpdateMap()
{
    auto now = std::chrono::system_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->previousMapUpdateTime);
    unsigned int ms = milliseconds.count();

    if (ms > this->mapUpdateRateMs)
    {
        this->updateMap();

        this->previousMapUpdateTime = now;
        return true;
    }
    return false;
}

const nav_msgs::OccupancyGrid &MapFuser::getMapMsg()
{
    // Fill message with required data.
    this->mapMsg.header.stamp = ros::Time::now();
    this->mapMsg.info.origin.position.x = -params.mapSizeMeters / 2.0;
    this->mapMsg.info.origin.position.y = -params.mapSizeMeters / 2.0;

    return this->mapMsg;
}

const geometry_msgs::TransformStamped &MapFuser::getTransformMsg()
{
    this->transformMsg.header.stamp = ros::Time::now();

    // Accommodate odometry and it's offset.
    this->transformMsg.transform.translation.x = this->latestOdomX - this->odomOffsetX;
    this->transformMsg.transform.translation.y = this->latestOdomY - this->odomOffsetY;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->latestOdomTheta - this->odomOffsetTheta);
    this->transformMsg.transform.rotation.x = q.x();
    this->transformMsg.transform.rotation.y = q.y();
    this->transformMsg.transform.rotation.z = q.z();
    this->transformMsg.transform.rotation.w = q.w();

    // The transform created above is valid for odom to map transform
    // but we need from map to odom.
    // So we need to invert it.
    tf::Transform tfTransform;
    tf::transformMsgToTF(this->transformMsg.transform, tfTransform);
    tf::transformTFToMsg(tfTransform.inverse(), this->transformMsg.transform);

    return this->transformMsg;
}