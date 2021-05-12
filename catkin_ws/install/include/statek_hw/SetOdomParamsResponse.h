// Generated by gencpp from file statek_hw/SetOdomParamsResponse.msg
// DO NOT EDIT!


#ifndef STATEK_HW_MESSAGE_SETODOMPARAMSRESPONSE_H
#define STATEK_HW_MESSAGE_SETODOMPARAMSRESPONSE_H


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
struct SetOdomParamsResponse_
{
  typedef SetOdomParamsResponse_<ContainerAllocator> Type;

  SetOdomParamsResponse_()
    : success(false)  {
    }
  SetOdomParamsResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetOdomParamsResponse_

typedef ::statek_hw::SetOdomParamsResponse_<std::allocator<void> > SetOdomParamsResponse;

typedef boost::shared_ptr< ::statek_hw::SetOdomParamsResponse > SetOdomParamsResponsePtr;
typedef boost::shared_ptr< ::statek_hw::SetOdomParamsResponse const> SetOdomParamsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator1> & lhs, const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator1> & lhs, const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace statek_hw

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "statek_hw/SetOdomParamsResponse";
  }

  static const char* value(const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success # Set to true on success.\n"
;
  }

  static const char* value(const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetOdomParamsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::statek_hw::SetOdomParamsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::statek_hw::SetOdomParamsResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATEK_HW_MESSAGE_SETODOMPARAMSRESPONSE_H