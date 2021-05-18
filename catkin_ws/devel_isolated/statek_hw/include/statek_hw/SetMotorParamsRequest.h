// Generated by gencpp from file statek_hw/SetMotorParamsRequest.msg
// DO NOT EDIT!


#ifndef STATEK_HW_MESSAGE_SETMOTORPARAMSREQUEST_H
#define STATEK_HW_MESSAGE_SETMOTORPARAMSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace statek_hw
{
template <class ContainerAllocator>
struct SetMotorParamsRequest_
{
  typedef SetMotorParamsRequest_<ContainerAllocator> Type;

  SetMotorParamsRequest_()
    : loop_update_rate_ms(0)
    , wheel_max_angular_velocity(0.0)
    , smoothing_factor(0.0)
    , kp(0.0)
    , ki(0.0)
    , kd(0.0)  {
    }
  SetMotorParamsRequest_(const ContainerAllocator& _alloc)
    : loop_update_rate_ms(0)
    , wheel_max_angular_velocity(0.0)
    , smoothing_factor(0.0)
    , kp(0.0)
    , ki(0.0)
    , kd(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _loop_update_rate_ms_type;
  _loop_update_rate_ms_type loop_update_rate_ms;

   typedef double _wheel_max_angular_velocity_type;
  _wheel_max_angular_velocity_type wheel_max_angular_velocity;

   typedef double _smoothing_factor_type;
  _smoothing_factor_type smoothing_factor;

   typedef double _kp_type;
  _kp_type kp;

   typedef double _ki_type;
  _ki_type ki;

   typedef double _kd_type;
  _kd_type kd;





  typedef boost::shared_ptr< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetMotorParamsRequest_

typedef ::statek_hw::SetMotorParamsRequest_<std::allocator<void> > SetMotorParamsRequest;

typedef boost::shared_ptr< ::statek_hw::SetMotorParamsRequest > SetMotorParamsRequestPtr;
typedef boost::shared_ptr< ::statek_hw::SetMotorParamsRequest const> SetMotorParamsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator1> & lhs, const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.loop_update_rate_ms == rhs.loop_update_rate_ms &&
    lhs.wheel_max_angular_velocity == rhs.wheel_max_angular_velocity &&
    lhs.smoothing_factor == rhs.smoothing_factor &&
    lhs.kp == rhs.kp &&
    lhs.ki == rhs.ki &&
    lhs.kd == rhs.kd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator1> & lhs, const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace statek_hw

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b8b46f7af46e051c167d322324998b79";
  }

  static const char* value(const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb8b46f7af46e051cULL;
  static const uint64_t static_value2 = 0x167d322324998b79ULL;
};

template<class ContainerAllocator>
struct DataType< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "statek_hw/SetMotorParamsRequest";
  }

  static const char* value(const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 loop_update_rate_ms # In ms. Set to 0 to ignore.\n"
"float64 wheel_max_angular_velocity # In rad/s. Set to negative to ignore.\n"
"float64 smoothing_factor # Between 0 and 1. Set to negative to ignore.\n"
"float64 kp # Set to negative to ignore.\n"
"float64 ki # Set to negative to ignore.\n"
"float64 kd # Set to negative to ignore.\n"
;
  }

  static const char* value(const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.loop_update_rate_ms);
      stream.next(m.wheel_max_angular_velocity);
      stream.next(m.smoothing_factor);
      stream.next(m.kp);
      stream.next(m.ki);
      stream.next(m.kd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMotorParamsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::statek_hw::SetMotorParamsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::statek_hw::SetMotorParamsRequest_<ContainerAllocator>& v)
  {
    s << indent << "loop_update_rate_ms: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.loop_update_rate_ms);
    s << indent << "wheel_max_angular_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.wheel_max_angular_velocity);
    s << indent << "smoothing_factor: ";
    Printer<double>::stream(s, indent + "  ", v.smoothing_factor);
    s << indent << "kp: ";
    Printer<double>::stream(s, indent + "  ", v.kp);
    s << indent << "ki: ";
    Printer<double>::stream(s, indent + "  ", v.ki);
    s << indent << "kd: ";
    Printer<double>::stream(s, indent + "  ", v.kd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATEK_HW_MESSAGE_SETMOTORPARAMSREQUEST_H
