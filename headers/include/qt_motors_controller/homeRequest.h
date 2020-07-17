// Generated by gencpp from file qt_motors_controller/homeRequest.msg
// DO NOT EDIT!


#ifndef QT_MOTORS_CONTROLLER_MESSAGE_HOMEREQUEST_H
#define QT_MOTORS_CONTROLLER_MESSAGE_HOMEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qt_motors_controller
{
template <class ContainerAllocator>
struct homeRequest_
{
  typedef homeRequest_<ContainerAllocator> Type;

  homeRequest_()
    : parts()  {
    }
  homeRequest_(const ContainerAllocator& _alloc)
    : parts(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _parts_type;
  _parts_type parts;





  typedef boost::shared_ptr< ::qt_motors_controller::homeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qt_motors_controller::homeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct homeRequest_

typedef ::qt_motors_controller::homeRequest_<std::allocator<void> > homeRequest;

typedef boost::shared_ptr< ::qt_motors_controller::homeRequest > homeRequestPtr;
typedef boost::shared_ptr< ::qt_motors_controller::homeRequest const> homeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qt_motors_controller::homeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qt_motors_controller::homeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace qt_motors_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'qt_motors_controller': ['/home/qtrobot/catkin_ws/src/qt_hardware_interface/controllers/qt_motors_controller/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qt_motors_controller::homeRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_motors_controller::homeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_motors_controller::homeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "61920c0acc31cd7d2b61692a8f73f78b";
  }

  static const char* value(const ::qt_motors_controller::homeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x61920c0acc31cd7dULL;
  static const uint64_t static_value2 = 0x2b61692a8f73f78bULL;
};

template<class ContainerAllocator>
struct DataType< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qt_motors_controller/homeRequest";
  }

  static const char* value(const ::qt_motors_controller::homeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string[] parts\n\
";
  }

  static const char* value(const ::qt_motors_controller::homeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.parts);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct homeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qt_motors_controller::homeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qt_motors_controller::homeRequest_<ContainerAllocator>& v)
  {
    s << indent << "parts[]" << std::endl;
    for (size_t i = 0; i < v.parts.size(); ++i)
    {
      s << indent << "  parts[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.parts[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // QT_MOTORS_CONTROLLER_MESSAGE_HOMEREQUEST_H
