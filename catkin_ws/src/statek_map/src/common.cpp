#include "../include/common.hpp"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

geometry_msgs::TransformStamped getTransform(const std::string &targetFrame, const std::string &sourceFrame, bool &ok)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener transformListener(tfBuffer);

    geometry_msgs::TransformStamped result;

    if (targetFrame == "" || sourceFrame == "")
    {
        ok = false;
        return result;
    }

    // Fixes extrapolation into the future in Rviz if run remotely.
    ros::Time now = ros::Time::now();
    ok = tfBuffer.canTransform(targetFrame, sourceFrame, now, ros::Duration(10.0));
    if (!ok)
        return result;

    ok = true;
    try
    {
        result = tfBuffer.lookupTransform(targetFrame, sourceFrame, now);
    }
    catch (tf::TransformException ex)
    {
        ok = false;
    }

    return result;
}