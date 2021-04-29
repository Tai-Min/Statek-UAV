#ifndef _ROS_SERVICE_SetImuParams_h
#define _ROS_SERVICE_SetImuParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_msgs
{

static const char SETIMUPARAMS[] = "statek_msgs/SetImuParams";

  class SetImuParamsRequest : public ros::Msg
  {
    public:
      typedef uint32_t _imu_update_rate_ms_type;
      _imu_update_rate_ms_type imu_update_rate_ms;
      float acc_bias[3];
      float gyro_bias[3];
      float mag_bias[3];
      float mag_scale[3];
      typedef float _magnetic_declination_type;
      _magnetic_declination_type magnetic_declination;

    SetImuParamsRequest():
      imu_update_rate_ms(0),
      acc_bias(),
      gyro_bias(),
      mag_bias(),
      mag_scale(),
      magnetic_declination(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->imu_update_rate_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->imu_update_rate_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->imu_update_rate_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->imu_update_rate_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_update_rate_ms);
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_bias[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->gyro_bias[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->mag_bias[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->mag_scale[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->magnetic_declination);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->imu_update_rate_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->imu_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->imu_update_rate_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->imu_update_rate_ms);
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_bias[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gyro_bias[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mag_bias[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mag_scale[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->magnetic_declination));
     return offset;
    }

    virtual const char * getType() override { return SETIMUPARAMS; };
    virtual const char * getMD5() override { return "01544b163fc15bc3cce2380ddafda3bc"; };

  };

  class SetImuParamsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetImuParamsResponse():
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

    virtual const char * getType() override { return SETIMUPARAMS; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetImuParams {
    public:
    typedef SetImuParamsRequest Request;
    typedef SetImuParamsResponse Response;
  };

}
#endif
