#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace
{
    const uint8_t AM4096_ABSOLUTE_ANGLE_ADDR = 33;
    const uint16_t AM4096_ANGLE_RESOLUTION = 0b0000111111111111;
    const float AM4096_ANGLE_MAX = 2 * M_PI; // In rad.

    const uint8_t AM4096_TACHO_ADDR = 35;
    const uint16_t AM4096_TACHO_RESOLUTION = 0b0000001111111111;
    const float AM4096_TACHO_MEASURING_RANGE = (960 / 60.0) * 2 * M_PI; // In rad/s.
}

class AM4096
{
private:
    TwoWire &i2c;
    uint8_t addr;

    float readMeasurement(uint8_t register_addr, uint16_t resolution, float range, bool & ok) const
    {
        ok = true;

        i2c.beginTransmission(this->addr);
        i2c.write(register_addr);               
        i2c.endTransmission(0);
        i2c.requestFrom((uint8_t)this->addr, (uint8_t)2);

        unsigned long long startTime = millis();
        while (!i2c.available())
        {
            unsigned long long currentTime = millis();

            // Millis overflow happened!
            // So handle it by restarting the timeout.
            // Shouldn't matter that much as it's like once per 50 days.
            if(startTime < currentTime){
                startTime = millis();
                currentTime = millis();
            }
            // Allow like 3ms to perform operation then timeout.
            else if(currentTime - startTime >= 3){
                ok = false;
                return 0;
            }
        }

        delayMicroseconds(50);

        uint8_t data[2] = {0};
        uint8_t i = 0;
        while (i2c.available())
        {
            data[i++] = i2c.read();
        }
        uint16_t val = (uint16_t)(resolution & (((uint16_t)data[0] << 8) | data[1]));
        
        return val / (float)resolution * range;
    }

public:
    AM4096(uint8_t _addr, TwoWire &tw = Wire) : i2c(tw), addr(_addr) {}

    float absolutePosition(bool &ok) const
    {
        return readMeasurement(AM4096_ABSOLUTE_ANGLE_ADDR, AM4096_ANGLE_RESOLUTION, AM4096_ANGLE_MAX, ok);
    }

    float velocity(bool &ok) const
    {
        return readMeasurement(AM4096_TACHO_ADDR, AM4096_TACHO_RESOLUTION, AM4096_TACHO_MEASURING_RANGE, ok);
    }
};