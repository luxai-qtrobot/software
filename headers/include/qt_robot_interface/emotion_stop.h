// Generated by gencpp from file qt_robot_interface/emotion_stop.msg
// DO NOT EDIT!


#ifndef QT_ROBOT_INTERFACE_MESSAGE_EMOTION_STOP_H
#define QT_ROBOT_INTERFACE_MESSAGE_EMOTION_STOP_H

#include <ros/service_traits.h>


#include <qt_robot_interface/emotion_stopRequest.h>
#include <qt_robot_interface/emotion_stopResponse.h>


namespace qt_robot_interface
{

struct emotion_stop
{

typedef emotion_stopRequest Request;
typedef emotion_stopResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct emotion_stop
} // namespace qt_robot_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qt_robot_interface::emotion_stop > {
  static const char* value()
  {
    return "3a1255d4d998bd4d6585c64639b5ee9a";
  }

  static const char* value(const ::qt_robot_interface::emotion_stop&) { return value(); }
};

template<>
struct DataType< ::qt_robot_interface::emotion_stop > {
  static const char* value()
  {
    return "qt_robot_interface/emotion_stop";
  }

  static const char* value(const ::qt_robot_interface::emotion_stop&) { return value(); }
};


// service_traits::MD5Sum< ::qt_robot_interface::emotion_stopRequest> should match 
// service_traits::MD5Sum< ::qt_robot_interface::emotion_stop > 
template<>
struct MD5Sum< ::qt_robot_interface::emotion_stopRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qt_robot_interface::emotion_stop >::value();
  }
  static const char* value(const ::qt_robot_interface::emotion_stopRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_robot_interface::emotion_stopRequest> should match 
// service_traits::DataType< ::qt_robot_interface::emotion_stop > 
template<>
struct DataType< ::qt_robot_interface::emotion_stopRequest>
{
  static const char* value()
  {
    return DataType< ::qt_robot_interface::emotion_stop >::value();
  }
  static const char* value(const ::qt_robot_interface::emotion_stopRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qt_robot_interface::emotion_stopResponse> should match 
// service_traits::MD5Sum< ::qt_robot_interface::emotion_stop > 
template<>
struct MD5Sum< ::qt_robot_interface::emotion_stopResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qt_robot_interface::emotion_stop >::value();
  }
  static const char* value(const ::qt_robot_interface::emotion_stopResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_robot_interface::emotion_stopResponse> should match 
// service_traits::DataType< ::qt_robot_interface::emotion_stop > 
template<>
struct DataType< ::qt_robot_interface::emotion_stopResponse>
{
  static const char* value()
  {
    return DataType< ::qt_robot_interface::emotion_stop >::value();
  }
  static const char* value(const ::qt_robot_interface::emotion_stopResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QT_ROBOT_INTERFACE_MESSAGE_EMOTION_STOP_H
