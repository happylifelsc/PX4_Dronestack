// Generated by gencpp from file dronestack/waypoint_navRequest.msg
// DO NOT EDIT!


#ifndef DRONESTACK_MESSAGE_WAYPOINT_NAVREQUEST_H
#define DRONESTACK_MESSAGE_WAYPOINT_NAVREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dronestack
{
template <class ContainerAllocator>
struct waypoint_navRequest_
{
  typedef waypoint_navRequest_<ContainerAllocator> Type;

  waypoint_navRequest_()
    : a(0.0)
    , b(0.0)
    , c(0.0)  {
    }
  waypoint_navRequest_(const ContainerAllocator& _alloc)
    : a(0.0)
    , b(0.0)
    , c(0.0)  {
  (void)_alloc;
    }



   typedef double _a_type;
  _a_type a;

   typedef double _b_type;
  _b_type b;

   typedef double _c_type;
  _c_type c;





  typedef boost::shared_ptr< ::dronestack::waypoint_navRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dronestack::waypoint_navRequest_<ContainerAllocator> const> ConstPtr;

}; // struct waypoint_navRequest_

typedef ::dronestack::waypoint_navRequest_<std::allocator<void> > waypoint_navRequest;

typedef boost::shared_ptr< ::dronestack::waypoint_navRequest > waypoint_navRequestPtr;
typedef boost::shared_ptr< ::dronestack::waypoint_navRequest const> waypoint_navRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dronestack::waypoint_navRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dronestack::waypoint_navRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dronestack::waypoint_navRequest_<ContainerAllocator1> & lhs, const ::dronestack::waypoint_navRequest_<ContainerAllocator2> & rhs)
{
  return lhs.a == rhs.a &&
    lhs.b == rhs.b &&
    lhs.c == rhs.c;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dronestack::waypoint_navRequest_<ContainerAllocator1> & lhs, const ::dronestack::waypoint_navRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dronestack

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dronestack::waypoint_navRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dronestack::waypoint_navRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dronestack::waypoint_navRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6ffac936c9b4e668039d250d853dc9b2";
  }

  static const char* value(const ::dronestack::waypoint_navRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6ffac936c9b4e668ULL;
  static const uint64_t static_value2 = 0x039d250d853dc9b2ULL;
};

template<class ContainerAllocator>
struct DataType< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dronestack/waypoint_navRequest";
  }

  static const char* value(const ::dronestack::waypoint_navRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 a\n"
"float64 b\n"
"float64 c\n"
;
  }

  static const char* value(const ::dronestack::waypoint_navRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
      stream.next(m.c);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct waypoint_navRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dronestack::waypoint_navRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dronestack::waypoint_navRequest_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<double>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<double>::stream(s, indent + "  ", v.b);
    s << indent << "c: ";
    Printer<double>::stream(s, indent + "  ", v.c);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DRONESTACK_MESSAGE_WAYPOINT_NAVREQUEST_H
