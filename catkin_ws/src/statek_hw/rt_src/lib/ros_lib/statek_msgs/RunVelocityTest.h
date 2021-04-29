#ifndef _ROS_SERVICE_RunVelocityTest_h
#define _ROS_SERVICE_RunVelocityTest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_msgs
{

static const char RUNVELOCITYTEST[] = "statek_msgs/RunVelocityTest";

  class RunVelocityTestRequest : public ros::Msg
  {
    public:
      typedef uint32_t _time_ms_type;
      _time_ms_type time_ms;

    RunVelocityTestRequest():
      time_ms(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_ms);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->time_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->time_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_ms);
     return offset;
    }

    virtual const char * getType() override { return RUNVELOCITYTEST; };
    virtual const char * getMD5() override { return "6301565e50e0b100332dde1661ab3ebc"; };

  };

  class RunVelocityTestResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef float _velocity_type;
      _velocity_type velocity;

    RunVelocityTestResponse():
      success(0),
      velocity(0)
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
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
     return offset;
    }

    virtual const char * getType() override { return RUNVELOCITYTEST; };
    virtual const char * getMD5() override { return "738dd6bd6e6687eab1176abf1ea50abc"; };

  };

  class RunVelocityTest {
    public:
    typedef RunVelocityTestRequest Request;
    typedef RunVelocityTestResponse Response;
  };

}
#endif
