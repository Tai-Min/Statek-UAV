// Generated by gencpp from file statek_hw/SetMotorParams.msg
// DO NOT EDIT!


#ifndef STATEK_HW_MESSAGE_SETMOTORPARAMS_H
#define STATEK_HW_MESSAGE_SETMOTORPARAMS_H

#include <ros/service_traits.h>


#include <statek_hw/SetMotorParamsRequest.h>
#include <statek_hw/SetMotorParamsResponse.h>


namespace statek_hw
{

struct SetMotorParams
{

typedef SetMotorParamsRequest Request;
typedef SetMotorParamsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetMotorParams
} // namespace statek_hw


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::statek_hw::SetMotorParams > {
  static const char* value()
  {
    return "f3ac856c8e72619c8db00906cbcd4cd3";
  }

  static const char* value(const ::statek_hw::SetMotorParams&) { return value(); }
};

template<>
struct DataType< ::statek_hw::SetMotorParams > {
  static const char* value()
  {
    return "statek_hw/SetMotorParams";
  }

  static const char* value(const ::statek_hw::SetMotorParams&) { return value(); }
};


// service_traits::MD5Sum< ::statek_hw::SetMotorParamsRequest> should match
// service_traits::MD5Sum< ::statek_hw::SetMotorParams >
template<>
struct MD5Sum< ::statek_hw::SetMotorParamsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::statek_hw::SetMotorParams >::value();
  }
  static const char* value(const ::statek_hw::SetMotorParamsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::statek_hw::SetMotorParamsRequest> should match
// service_traits::DataType< ::statek_hw::SetMotorParams >
template<>
struct DataType< ::statek_hw::SetMotorParamsRequest>
{
  static const char* value()
  {
    return DataType< ::statek_hw::SetMotorParams >::value();
  }
  static const char* value(const ::statek_hw::SetMotorParamsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::statek_hw::SetMotorParamsResponse> should match
// service_traits::MD5Sum< ::statek_hw::SetMotorParams >
template<>
struct MD5Sum< ::statek_hw::SetMotorParamsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::statek_hw::SetMotorParams >::value();
  }
  static const char* value(const ::statek_hw::SetMotorParamsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::statek_hw::SetMotorParamsResponse> should match
// service_traits::DataType< ::statek_hw::SetMotorParams >
template<>
struct DataType< ::statek_hw::SetMotorParamsResponse>
{
  static const char* value()
  {
    return DataType< ::statek_hw::SetMotorParams >::value();
  }
  static const char* value(const ::statek_hw::SetMotorParamsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // STATEK_HW_MESSAGE_SETMOTORPARAMS_H
