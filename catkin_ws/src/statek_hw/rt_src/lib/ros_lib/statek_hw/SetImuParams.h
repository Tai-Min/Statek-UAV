#ifndef _ROS_SERVICE_SetImuParams_h
#define _ROS_SERVICE_SetImuParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_hw
{

static const char SETIMUPARAMS[] = "statek_hw/SetImuParams";

  class SetImuParamsRequest : public ros::Msg
  {
    public:
      typedef uint32_t _imu_update_rate_ms_type;
      _imu_update_rate_ms_type imu_update_rate_ms;
      float acc_bias[3];
      float gyro_bias[3];
      float mag_bias[3];
      float mag_scale[3];
      int16_t mag_dec[3];

    SetImuParamsRequest():
      imu_update_rate_ms(0),
      acc_bias(),
      gyro_bias(),
      mag_bias(),
      mag_scale(),
      mag_dec()
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
      for( uint32_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_mag_deci;
      u_mag_deci.real = this->mag_dec[i];
      *(outbuffer + offset + 0) = (u_mag_deci.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_deci.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mag_dec[i]);
      }
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
      for( uint32_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_mag_deci;
      u_mag_deci.base = 0;
      u_mag_deci.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_deci.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mag_dec[i] = u_mag_deci.real;
      offset += sizeof(this->mag_dec[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return SETIMUPARAMS; };
    virtual const char * getMD5() override { return "5b52e44d9a6cc076ecccb46df6d4a341"; };

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
