#ifndef _ROS_SERVICE_SetOdomParams_h
#define _ROS_SERVICE_SetOdomParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_hw
{

static const char SETODOMPARAMS[] = "statek_hw/SetOdomParams";

  class SetOdomParamsRequest : public ros::Msg
  {
    public:
      typedef uint32_t _odom_update_rate_ms_type;
      _odom_update_rate_ms_type odom_update_rate_ms;
      typedef float _distance_between_wheels_type;
      _distance_between_wheels_type distance_between_wheels;
      typedef float _wheel_radius_type;
      _wheel_radius_type wheel_radius;

    SetOdomParamsRequest():
      odom_update_rate_ms(0),
      distance_between_wheels(0),
      wheel_radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->odom_update_rate_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->odom_update_rate_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->odom_update_rate_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->odom_update_rate_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->odom_update_rate_ms);
      offset += serializeAvrFloat64(outbuffer + offset, this->distance_between_wheels);
      offset += serializeAvrFloat64(outbuffer + offset, this->wheel_radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->odom_update_rate_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->odom_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->odom_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->odom_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->odom_update_rate_ms);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance_between_wheels));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->wheel_radius));
     return offset;
    }

    virtual const char * getType() override { return SETODOMPARAMS; };
    virtual const char * getMD5() override { return "c2c1b0d7cf2d7ec7ca5237bbb9467136"; };

  };

  class SetOdomParamsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetOdomParamsResponse():
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

    virtual const char * getType() override { return SETODOMPARAMS; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetOdomParams {
    public:
    typedef SetOdomParamsRequest Request;
    typedef SetOdomParamsResponse Response;
  };

}
#endif
