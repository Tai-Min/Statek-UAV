#ifndef _ROS_statek_hw_Encoder_h
#define _ROS_statek_hw_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace statek_hw
{

  class Encoder : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _acceleration_type;
      _acceleration_type acceleration;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _position_type;
      _position_type position;

    Encoder():
      header(),
      acceleration(0),
      velocity(0),
      position(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acceleration));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
     return offset;
    }

    virtual const char * getType() override { return "statek_hw/Encoder"; };
    virtual const char * getMD5() override { return "72b8bd04d914e248fb0847af003c28c1"; };

  };

}
#endif
