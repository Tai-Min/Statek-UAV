#pragma once

#include <sensor_msgs/BatteryState.h>
#include "../ros_handlers/ros.h"

class BatterySupervisor
{
private:
    //hardware
    AnalogIn batteryVoltage;

    // ros stuff
    sensor_msgs::BatteryState batteryStateMsg;
    ros::Publisher batteryStatePublisher;

    void batterySupervisorThreadFcn()
    {
        nh.advertise(this->batteryStatePublisher);

        uint32_t seq = 0;
        while (true)
        {
            this->batteryStateMsg.header.seq = seq;
            this->batteryStateMsg.header.stamp = nh.now();

            seq++;
            Thread::wait(1000);
        }
    }

public:
    BatterySupervisor(const Gpio &adcGpio, const char *batteryStateTopic, const char *batteryFrame)
        : batteryVoltage(adcGpio), batteryStatePublisher(batteryStateTopic, &this->batteryStateMsg)
    {
        this->batteryStateMsg.header.frame_id = batteryFrame;
        this->batteryStateMsg.current = nan;
        this->batteryStateMsg.charge = nan;
        this->batteryStateMsg.capacity = nan;
        this->batteryStateMsg.power_supply_status = this->batteryStateMsg.POWER_SUPPLY_STATUS_DISCHARGING;
        this->batteryStateMsg.power_supply_status = this->batteryStateMsg.POWER_SUPPLY_HEALTH_UNKNOWN;
        this->batteryStateMsg.power_supply_technology = this->batteryStateMsg.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    }
    void start();
};