#pragma once

#include <mbed.h>

#define AM4096_ABSOLUTE_ANGLE_ADDR 33
#define AM4096_ANGLE_RESOLUTION 4096
#define AM4096_ANGLE_BITMASK 0b0000111111111111
#define AM4096_ANGLE_MAX 2 * M_PI // in [rad]

#define AM4096_TACHO_ADDR 35
#define AM4096_TACHO_RESOLUTION 1024
#define AM4096_TACHO_BITMASK 0b0000001111111111
#define AM4096_TACHO_MEASURING_RANGE 960 / 60.0 * 2 * M_PI // in [rad/s]

class AM4096
{
private:
    I2C &i2c;
    uint8_t addr;
    bool readZero = true;
    double zeroPosition;

    double readEncoder(uint8_t register_addr, uint16_t resolution, uint16_t bitmask, uint16_t range)
    {
        char data[2];
        data[0] = register_addr;

        i2c.write(addr, data, 1, true);
        i2c.read(addr, data, 2);

        uint16_t val = bitmask & (data[0] << 8 | data[1]);

        return val / (double)resolution * range;
    }

    double readAngle(uint8_t register_addr)
    {
        return readEncoder(register_addr, AM4096_ANGLE_RESOLUTION, AM4096_ANGLE_BITMASK, AM4096_ANGLE_MAX);
    }

public:
    AM4096(I2C &_i2c, uint8_t _addr) : i2c(_i2c), addr(_addr << 1) {}

    double angle()
    {
        double reading = readAngle(AM4096_ABSOLUTE_ANGLE_ADDR);

        // treat first reading as zero position
        if (readZero)
        {
            readZero = false;
            zeroPosition = reading;
            return 0.0;
        }

        return zeroPosition - reading;
    }

    double velocity()
    {
        return readEncoder(AM4096_TACHO_ADDR, AM4096_TACHO_RESOLUTION, AM4096_TACHO_BITMASK, AM4096_TACHO_MEASURING_RANGE);
    }
};