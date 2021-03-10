#include "motor_controller.hpp"

#include "../ros_handlers/ros_handlers.hpp"
#include "statek_msgs/Encoder.h"

MotorController::MotorController(const MotorGpio &motorGPIO, I2C &encoderI2C, uint8_t encoderAddr, Side _side, bool _reverseEncoder)
    : motor(motorGPIO.en, motorGPIO.cw, motorGPIO.ccw, motorGPIO.pwm), encoder(encoderI2C, encoderAddr), reverseEncoder(_reverseEncoder), side(_side)
{
}

void MotorController::controlLoopThreadFcn()
{
    unsigned long long cntr = 0;
    statek_msgs::Encoder encoderMsg;
    ros::Publisher rawEncoderPublisher(this->side == Side::LEFT ? "motors/left/encoder/raw" : "motors/right/encoder/raw", &encoderMsg);
    nh.advertise(rawEncoderPublisher);
    ros::Publisher filteredEncoderPublisher(this->side == Side::LEFT ? "motors/left/encoder/filtered" : "motors/right/encoder/filtered", &encoderMsg);
    nh.advertise(filteredEncoderPublisher);

    motor.enable();
    //motor.write(0.3);
    while (true)
    {
        encoderMsg.header.seq = cntr;
        encoderMsg.header.stamp = nh.now();
        
        encoderMsg.velocity = this->encoder.velocity();
        Thread::wait(1);

        encoderMsg.position = this->encoder.angle();
        if(this->reverseEncoder)
            encoderMsg.position = 2 * M_PI - encoderMsg.position;
        Thread::wait(1);

        rawEncoderPublisher.publish(&encoderMsg);
        filteredEncoderPublisher.publish(&encoderMsg);

        cntr++;
        Thread::wait(48);
    }
}

void MotorController::start()
{
    this->controlLoopThread.start(callback(this, &MotorController::controlLoopThreadFcn));
}

void MotorController::setVelocity(double sp){
    this->setpoint = sp;
}

void MotorController::setControlMode(ControlMode cm) {
    this->controlMode = cm;
}