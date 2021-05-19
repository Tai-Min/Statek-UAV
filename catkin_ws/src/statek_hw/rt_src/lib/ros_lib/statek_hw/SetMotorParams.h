#ifndef _ROS_SERVICE_SetMotorParams_h
#define _ROS_SERVICE_SetMotorParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_hw
{

static const char SETMOTORPARAMS[] = "statek_hw/SetMotorParams";

  class SetMotorParamsRequest : public ros::Msg
  {
    public:
      typedef uint32_t _loop_update_rate_ms_type;
      _loop_update_rate_ms_type loop_update_rate_ms;
      typedef float _wheel_max_angular_velocity_type;
      _wheel_max_angular_velocity_type wheel_max_angular_velocity;
      typedef float _smoothing_factor_type;
      _smoothing_factor_type smoothing_factor;
      typedef float _kp_type;
      _kp_type kp;
      typedef float _ki_type;
      _ki_type ki;
      typedef float _kd_type;
      _kd_type kd;

    SetMotorParamsRequest():
      loop_update_rate_ms(0),
      wheel_max_angular_velocity(0),
      smoothing_factor(0),
      kp(0),
      ki(0),
      kd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->loop_update_rate_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->loop_update_rate_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->loop_update_rate_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->loop_update_rate_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->loop_update_rate_ms);
      union {
        float real;
        uint32_t base;
      } u_wheel_max_angular_velocity;
      u_wheel_max_angular_velocity.real = this->wheel_max_angular_velocity;
      *(outbuffer + offset + 0) = (u_wheel_max_angular_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_max_angular_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_max_angular_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_max_angular_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_max_angular_velocity);
      union {
        float real;
        uint32_t base;
      } u_smoothing_factor;
      u_smoothing_factor.real = this->smoothing_factor;
      *(outbuffer + offset + 0) = (u_smoothing_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_smoothing_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_smoothing_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_smoothing_factor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->smoothing_factor);
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->loop_update_rate_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->loop_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->loop_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->loop_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->loop_update_rate_ms);
      union {
        float real;
        uint32_t base;
      } u_wheel_max_angular_velocity;
      u_wheel_max_angular_velocity.base = 0;
      u_wheel_max_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_max_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_max_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_max_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_max_angular_velocity = u_wheel_max_angular_velocity.real;
      offset += sizeof(this->wheel_max_angular_velocity);
      union {
        float real;
        uint32_t base;
      } u_smoothing_factor;
      u_smoothing_factor.base = 0;
      u_smoothing_factor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_smoothing_factor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_smoothing_factor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_smoothing_factor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->smoothing_factor = u_smoothing_factor.real;
      offset += sizeof(this->smoothing_factor);
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.base = 0;
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki = u_ki.real;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.base = 0;
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd = u_kd.real;
      offset += sizeof(this->kd);
     return offset;
    }

    virtual const char * getType() override { return SETMOTORPARAMS; };
    virtual const char * getMD5() override { return "b2fa7ecfe95991cb18a44515bf75cd52"; };

  };

  class SetMotorParamsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetMotorParamsResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return SETMOTORPARAMS; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetMotorParams {
    public:
    typedef SetMotorParamsRequest Request;
    typedef SetMotorParamsResponse Response;
  };

}
#endif
