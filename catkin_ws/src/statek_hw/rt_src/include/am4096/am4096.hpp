#pragma once

#include <mbed.h>

namespace
{
    const uint8_t AM4096_ABSOLUTE_ANGLE_ADDR = 33;
    const uint16_t AM4096_ANGLE_RESOLUTION = 4096;
    const uint16_t AM4096_ANGLE_BITMASK = 0b0000111111111111;
    const double AM4096_ANGLE_MAX = 2 * M_PI; // in [rad]

    const uint8_t AM4096_TACHO_ADDR = 35;
    const uint16_t AM4096_TACHO_RESOLUTION = 1024;
    const uint16_t AM4096_TACHO_BITMASK = 0b0000001111111111;
    const double AM4096_TACHO_MEASURING_RANGE = (960 / 60.0) * 2 * M_PI; // in [rad/s]
}

class AM4096
{
private:
    I2C &i2c;
    uint8_t addr;
    bool readZero = true;
    double zeroPosition;

    double readMeasurement(uint8_t register_addr, uint16_t resolution, uint16_t bitmask, double range)
    {
        char data[2];
        data[0] = register_addr;

        this->i2c.write(this->addr, data, 1, true);
        this->i2c.read(this->addr, data, 2);

        uint16_t val = bitmask & (data[0] << 8 | data[1]);

        return val / (double)resolution * range;
    }

    double readAngle(uint8_t register_addr)
    {
        return readMeasurement(register_addr, AM4096_ANGLE_RESOLUTION, AM4096_ANGLE_BITMASK, AM4096_ANGLE_MAX);
    }

public:
    AM4096(I2C &_i2c, uint8_t _addr) : i2c(_i2c), addr(_addr << 1) {}

    double position()
    {
        if(this->readZero){
            this->readZero = false;
            this->zeroPosition = readAngle(AM4096_ABSOLUTE_ANGLE_ADDR);
            return 0;
        }
        return this->zeroPosition - readAngle(AM4096_ABSOLUTE_ANGLE_ADDR);
    }

    double velocity()
    {
        return readMeasurement(AM4096_TACHO_ADDR, AM4096_TACHO_RESOLUTION, AM4096_TACHO_BITMASK, AM4096_TACHO_MEASURING_RANGE);
    }
};