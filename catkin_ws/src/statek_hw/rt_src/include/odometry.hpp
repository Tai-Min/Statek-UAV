#pragma once

class Odometry {

    float wheelRadius = 0;
    float latestLeftWheenDistance;
    float latestRightWheelDistance;
    
private:
    void update();
public:
    bool tryUpdate();
};