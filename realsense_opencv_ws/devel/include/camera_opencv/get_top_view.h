// Generated by gencpp from file camera_opencv/get_top_view.msg
// DO NOT EDIT!


#ifndef CAMERA_OPENCV_MESSAGE_GET_TOP_VIEW_H
#define CAMERA_OPENCV_MESSAGE_GET_TOP_VIEW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace camera_opencv
{
template <class ContainerAllocator>
struct get_top_view_
{
  typedef get_top_view_<ContainerAllocator> Type;

  get_top_view_()
    {
    }
  get_top_view_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::camera_opencv::get_top_view_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::camera_opencv::get_top_view_<ContainerAllocator> const> ConstPtr;

}; // struct get_top_view_

typedef ::camera_opencv::get_top_view_<std::allocator<void> > get_top_view;

typedef boost::shared_ptr< ::camera_opencv::get_top_view > get_top_viewPtr;
typedef boost::shared_ptr< ::camera_opencv::get_top_view const> get_top_viewConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::camera_opencv::get_top_view_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::camera_opencv::get_top_view_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace camera_opencv

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'camera_opencv': ['/home/songsong/realsense_opencv_ws/src/camera_opencv/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::camera_opencv::get_top_view_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camera_opencv::get_top_view_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camera_opencv::get_top_view_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camera_opencv::get_top_view_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera_opencv::get_top_view_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera_opencv::get_top_view_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::camera_opencv::get_top_view_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::camera_opencv::get_top_view_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::camera_opencv::get_top_view_<ContainerAllocator> >
{
  static const char* value()
  {
    return "camera_opencv/get_top_view";
  }

  static const char* value(const ::camera_opencv::get_top_view_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::camera_opencv::get_top_view_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::camera_opencv::get_top_view_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::camera_opencv::get_top_view_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct get_top_view_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::camera_opencv::get_top_view_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::camera_opencv::get_top_view_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CAMERA_OPENCV_MESSAGE_GET_TOP_VIEW_H
