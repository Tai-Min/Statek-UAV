#pragma once
#include "ros.h"

extern bool serviceInProgress; //!< Check this flag before running requested service. If true, stop the service, else set it to true while service is running.
extern ros::NodeHandle nh; //!< Node handle used through the program.