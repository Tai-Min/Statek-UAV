#ifndef _ROS_statek_msgs_Velocity_h
#define _ROS_statek_msgs_Velocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_msgs
{

  class Velocity : public ros::Msg
  {
    public:
      typedef float _left_type;
      _left_type left;
      typedef float _right_type;
      _right_type right;

    Velocity():
      left(0),
      right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->left);
      offset += serializeAvrFloat64(outbuffer + offset, this->right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right));
     return offset;
    }

    virtual const char * getType() override { return "statek_msgs/Velocity"; };
    virtual const char * getMD5() override { return "50c2436c38cded221d061b57126c4e40"; };

  };

}
#endif
