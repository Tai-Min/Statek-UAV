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
    TwoWire &i2c; //!< Reference to used i2c object.
    uint8_t addr; //!< Encoder's address

    /**
     * @brief Read one of the measurements from the encoder. Position or velocity.
     * @param register_addr Address of the register to read.
     * @param resolution Resolution of the measurement. 
     * @param range Measurement range (0 to range) in desired units.
     * @param ok Set to true if measurement succeed, false otherwise.
     * @return Measurement scaled to desired range or 0 with ok = false if read failed.
     */
    float readMeasurement(uint8_t register_addr, uint16_t resolution, float range, bool & ok) const
    {
        ok = true;

        i2c.beginTransmission(this->addr);
        i2c.write(register_addr);               
        uint8_t res = i2c.endTransmission(0);
        if(res){
            ok = false;
            return 0;
        }
        i2c.requestFrom((uint8_t)this->addr, (uint8_t)2);

        unsigned long startTime = millis();
        while (!i2c.available())
        {
            unsigned long currentTime = millis();

            // Millis overflow happened!
            // So handle it by restarting the timeout.
            // Shouldn't matter that much as it's like once per 50 days.
            if(currentTime < startTime){
                startTime = millis();
                currentTime = millis();
            }
            // Allow like 3ms to perform operation then timeout.
            else if(currentTime - startTime >= 3){
                ok = false;
                return 0;
            }
        }

        delayMicroseconds(5);

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
    /**
     * @brief Class constructor.
     * @param _addr Address of the encoder.
     * @param tw Wire interface to read from.
     */
    AM4096(uint8_t _addr, TwoWire &tw = Wire) : i2c(tw), addr(_addr) {}

    /**
     * @brief Read absolute position of the encoder.
     * @param ok Set to true if measurement succeed, false otherwise.
     * @return Absolute position in radians or 0 if ok = false.
     */
    float absolutePosition(bool &ok) const
    {
        return readMeasurement(AM4096_ABSOLUTE_ANGLE_ADDR, AM4096_ANGLE_RESOLUTION, AM4096_ANGLE_MAX, ok);
    }

    /**
     * @brief Read velocity from tachometer.
     * @param ok Set to true if measurement succeed, false otherwise.
     * @return Velocity in radians per second or 0 if ok = false.
     */
    float velocity(bool &ok) const
    {
        return readMeasurement(AM4096_TACHO_ADDR, AM4096_TACHO_RESOLUTION, AM4096_TACHO_MEASURING_RANGE, ok);
    }
};