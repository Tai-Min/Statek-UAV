#pragma once

#include "../lib/ros_lib/ros/node_handle.h"
#include "../lib/ros_lib/ArduinoHardware.h"

namespace ros
{
    // Publishers, subscribers, input buffer, output buffer
    typedef NodeHandle_<ArduinoHardware, 25, 25, 1024, 1024> NodeHandle;
}