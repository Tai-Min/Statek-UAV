// Generated by gencpp from file statek_hw/SetOdomParamsRequest.msg
// DO NOT EDIT!


#ifndef STATEK_HW_MESSAGE_SETODOMPARAMSREQUEST_H
#define STATEK_HW_MESSAGE_SETODOMPARAMSREQUEST_H


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
struct SetOdomParamsRequest_
{
  typedef SetOdomParamsRequest_<ContainerAllocator> Type;

  SetOdomParamsRequest_()
    : odom_update_rate_ms(0)
    , distance_between_wheels(0.0)
    , wheel_radius(0.0)  {
    }
  SetOdomParamsRequest_(const ContainerAllocator& _alloc)
    : odom_update_rate_ms(0)
    , distance_between_wheels(0.0)
    , wheel_radius(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _odom_update_rate_ms_type;
  _odom_update_rate_ms_type odom_update_rate_ms;

   typedef double _distance_between_wheels_type;
  _distance_between_wheels_type distance_between_wheels;

   typedef double _wheel_radius_type;
  _wheel_radius_type wheel_radius;





  typedef boost::shared_ptr< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetOdomParamsRequest_

typedef ::statek_hw::SetOdomParamsRequest_<std::allocator<void> > SetOdomParamsRequest;

typedef boost::shared_ptr< ::statek_hw::SetOdomParamsRequest > SetOdomParamsRequestPtr;
typedef boost::shared_ptr< ::statek_hw::SetOdomParamsRequest const> SetOdomParamsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator1> & lhs, const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.odom_update_rate_ms == rhs.odom_update_rate_ms &&
    lhs.distance_between_wheels == rhs.distance_between_wheels &&
    lhs.wheel_radius == rhs.wheel_radius;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator1> & lhs, const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace statek_hw

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c2c1b0d7cf2d7ec7ca5237bbb9467136";
  }

  static const char* value(const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc2c1b0d7cf2d7ec7ULL;
  static const uint64_t static_value2 = 0xca5237bbb9467136ULL;
};

template<class ContainerAllocator>
struct DataType< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "statek_hw/SetOdomParamsRequest";
  }

  static const char* value(const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 odom_update_rate_ms # Set to 0 to ignore.\n"
"float64 distance_between_wheels # Set to negative to ignore.\n"
"float64 wheel_radius # Set to negative to ignore.\n"
;
  }

  static const char* value(const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.odom_update_rate_ms);
      stream.next(m.distance_between_wheels);
      stream.next(m.wheel_radius);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetOdomParamsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::statek_hw::SetOdomParamsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::statek_hw::SetOdomParamsRequest_<ContainerAllocator>& v)
  {
    s << indent << "odom_update_rate_ms: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.odom_update_rate_ms);
    s << indent << "distance_between_wheels: ";
    Printer<double>::stream(s, indent + "  ", v.distance_between_wheels);
    s << indent << "wheel_radius: ";
    Printer<double>::stream(s, indent + "  ", v.wheel_radius);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATEK_HW_MESSAGE_SETODOMPARAMSREQUEST_H