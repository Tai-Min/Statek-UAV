#ifndef _ROS_SERVICE_GeoToEnu_h
#define _ROS_SERVICE_GeoToEnu_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_map
{

static const char GEOTOENU[] = "statek_map/GeoToEnu";

  class GeoToEnuRequest : public ros::Msg
  {
    public:
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;

    GeoToEnuRequest():
      latitude(0),
      longitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
     return offset;
    }

    virtual const char * getType() override { return GEOTOENU; };
    virtual const char * getMD5() override { return "680c6dc7da65a2421a822205dcbdb600"; };

  };

  class GeoToEnuResponse : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;

    GeoToEnuResponse():
      x(0),
      y(0),
      z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
     return offset;
    }

    virtual const char * getType() override { return GEOTOENU; };
    virtual const char * getMD5() override { return "4a842b65f413084dc2b10fb484ea7f17"; };

  };

  class GeoToEnu {
    public:
    typedef GeoToEnuRequest Request;
    typedef GeoToEnuResponse Response;
  };

}
#endif
