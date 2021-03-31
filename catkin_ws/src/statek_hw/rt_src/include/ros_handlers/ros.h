#pragma once

#include <ros/node_handle.h>
#include <MbedHardware.h>

namespace ros
{
  // default is 25, 25, 512, 512  
  typedef NodeHandle_<MbedHardware, 10, 10, 1024, 1024> NodeHandle;
}