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
      offset += serializeAvrFloat64(outbuffer + offset, this->wheel_max_angular_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->smoothing_factor);
      offset += serializeAvrFloat64(outbuffer + offset, this->kp);
      offset += serializeAvrFloat64(outbuffer + offset, this->ki);
      offset += serializeAvrFloat64(outbuffer + offset, this->kd);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->wheel_max_angular_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->smoothing_factor));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ki));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kd));
     return offset;
    }

    virtual const char * getType() override { return SETMOTORPARAMS; };
    virtual const char * getMD5() override { return "b8b46f7af46e051c167d322324998b79"; };

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
