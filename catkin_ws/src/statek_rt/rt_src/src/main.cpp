#include <mbed.h>
//#include <ros.h>
//#include <std_msgs/String.h>

#include "../include/motor/motor.hpp"
#include "../include/am4096/am4096.hpp"

Serial pc(USBTX, USBRX);

#define D_SDA PB_9
#define D_SCL PB_8

I2C i2c(D_SDA, D_SCL);
Motor motorRight(PA_0, PA_8, PA_9, PB_4);
Motor motorLeft(PA_1, PB_5, PC_7, PB_10);
AM4096 encoderRight(i2c, 30);
AM4096 encoderLeft(i2c, 60);

int main()
{
  motorRight.enable();
  motorLeft.enable();

  motorRight.write(-0.0f);
  motorLeft.write(-0.0f);

  while (true)
  {
    double angleRight = encoderRight.angle();
    double angleLeft= encoderLeft.angle();
    double velocityRight = encoderRight.velocity();
    double velocityLeft = encoderLeft.velocity();

    pc.printf("Left: (angle: %f, velocity: %f), Right: (angle: %f, velocity: %f)\n", angleLeft, velocityLeft, angleRight, velocityRight);

    wait_ms(100.0f);
  }
}