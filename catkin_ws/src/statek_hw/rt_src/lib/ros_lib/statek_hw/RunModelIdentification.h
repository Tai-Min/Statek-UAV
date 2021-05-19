#ifndef _ROS_SERVICE_RunModelIdentification_h
#define _ROS_SERVICE_RunModelIdentification_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace statek_hw
{

static const char RUNMODELIDENTIFICATION[] = "statek_hw/RunModelIdentification";

  class RunModelIdentificationRequest : public ros::Msg
  {
    public:
      typedef uint32_t _identification_time_ms_type;
      _identification_time_ms_type identification_time_ms;

    RunModelIdentificationRequest():
      identification_time_ms(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->identification_time_ms >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->identification_time_ms >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->identification_time_ms >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->identification_time_ms >> (8 * 3)) & 0xFF;
      offset += sizeof(this->identification_time_ms);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->identification_time_ms =  ((uint32_t) (*(inbuffer + offset)));
      this->identification_time_ms |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->identification_time_ms |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->identification_time_ms |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->identification_time_ms);
     return offset;
    }

    virtual const char * getType() override { return RUNMODELIDENTIFICATION; };
    virtual const char * getMD5() override { return "1a1d96b6498339e94d9e98e6c92d5137"; };

  };

  class RunModelIdentificationResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef float _sampling_time_type;
      _sampling_time_type sampling_time;
      float samples[100];

    RunModelIdentificationResponse():
      success(0),
      sampling_time(0),
      samples()
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
      union {
        float real;
        uint32_t base;
      } u_sampling_time;
      u_sampling_time.real = this->sampling_time;
      *(outbuffer + offset + 0) = (u_sampling_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sampling_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sampling_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sampling_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sampling_time);
      for( uint32_t i = 0; i < 100; i++){
      union {
        float real;
        uint32_t base;
      } u_samplesi;
      u_samplesi.real = this->samples[i];
      *(outbuffer + offset + 0) = (u_samplesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_samplesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_samplesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_samplesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->samples[i]);
      }
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
      union {
        float real;
        uint32_t base;
      } u_sampling_time;
      u_sampling_time.base = 0;
      u_sampling_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sampling_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sampling_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sampling_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sampling_time = u_sampling_time.real;
      offset += sizeof(this->sampling_time);
      for( uint32_t i = 0; i < 100; i++){
      union {
        float real;
        uint32_t base;
      } u_samplesi;
      u_samplesi.base = 0;
      u_samplesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_samplesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_samplesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_samplesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->samples[i] = u_samplesi.real;
      offset += sizeof(this->samples[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return RUNMODELIDENTIFICATION; };
    virtual const char * getMD5() override { return "e522fddbf776342653ff430de909a94e"; };

  };

  class RunModelIdentification {
    public:
    typedef RunModelIdentificationRequest Request;
    typedef RunModelIdentificationResponse Response;
  };

}
#endif
