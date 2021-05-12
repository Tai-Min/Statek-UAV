// Generated by gencpp from file statek_hw/RunImuCalibrationRequest.msg
// DO NOT EDIT!


#ifndef STATEK_HW_MESSAGE_RUNIMUCALIBRATIONREQUEST_H
#define STATEK_HW_MESSAGE_RUNIMUCALIBRATIONREQUEST_H


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
struct RunImuCalibrationRequest_
{
  typedef RunImuCalibrationRequest_<ContainerAllocator> Type;

  RunImuCalibrationRequest_()
    {
    }
  RunImuCalibrationRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RunImuCalibrationRequest_

typedef ::statek_hw::RunImuCalibrationRequest_<std::allocator<void> > RunImuCalibrationRequest;

typedef boost::shared_ptr< ::statek_hw::RunImuCalibrationRequest > RunImuCalibrationRequestPtr;
typedef boost::shared_ptr< ::statek_hw::RunImuCalibrationRequest const> RunImuCalibrationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace statek_hw

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "statek_hw/RunImuCalibrationRequest";
  }

  static const char* value(const ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RunImuCalibrationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::statek_hw::RunImuCalibrationRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // STATEK_HW_MESSAGE_RUNIMUCALIBRATIONREQUEST_H