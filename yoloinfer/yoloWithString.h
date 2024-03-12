// Generated by gencpp from file yoloinfer/yoloWithString.msg
// DO NOT EDIT!


#ifndef YOLOINFER_MESSAGE_YOLOWITHSTRING_H
#define YOLOINFER_MESSAGE_YOLOWITHSTRING_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

namespace yoloinfer
{
template <class ContainerAllocator>
struct yoloWithString_
{
  typedef yoloWithString_<ContainerAllocator> Type;

  yoloWithString_()
    : image()
    , custom_string()  {
    }
  yoloWithString_(const ContainerAllocator& _alloc)
    : image(_alloc)
    , custom_string(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _image_type;
  _image_type image;

   typedef  ::std_msgs::String_<ContainerAllocator>  _custom_string_type;
  _custom_string_type custom_string;





  typedef boost::shared_ptr< ::yoloinfer::yoloWithString_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yoloinfer::yoloWithString_<ContainerAllocator> const> ConstPtr;

}; // struct yoloWithString_

typedef ::yoloinfer::yoloWithString_<std::allocator<void> > yoloWithString;

typedef boost::shared_ptr< ::yoloinfer::yoloWithString > yoloWithStringPtr;
typedef boost::shared_ptr< ::yoloinfer::yoloWithString const> yoloWithStringConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::yoloinfer::yoloWithString_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::yoloinfer::yoloWithString_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::yoloinfer::yoloWithString_<ContainerAllocator1> & lhs, const ::yoloinfer::yoloWithString_<ContainerAllocator2> & rhs)
{
  return lhs.image == rhs.image &&
    lhs.custom_string == rhs.custom_string;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::yoloinfer::yoloWithString_<ContainerAllocator1> & lhs, const ::yoloinfer::yoloWithString_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace yoloinfer

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::yoloinfer::yoloWithString_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yoloinfer::yoloWithString_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yoloinfer::yoloWithString_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yoloinfer::yoloWithString_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yoloinfer::yoloWithString_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yoloinfer::yoloWithString_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::yoloinfer::yoloWithString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e128c42d87e4fde93eca4698b44a09c8";
  }

  static const char* value(const ::yoloinfer::yoloWithString_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe128c42d87e4fde9ULL;
  static const uint64_t static_value2 = 0x3eca4698b44a09c8ULL;
};

template<class ContainerAllocator>
struct DataType< ::yoloinfer::yoloWithString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "yoloinfer/yoloWithString";
  }

  static const char* value(const ::yoloinfer::yoloWithString_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::yoloinfer::yoloWithString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/Image image\n"
"std_msgs/String custom_string\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::yoloinfer::yoloWithString_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::yoloinfer::yoloWithString_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.image);
      stream.next(m.custom_string);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct yoloWithString_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yoloinfer::yoloWithString_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::yoloinfer::yoloWithString_<ContainerAllocator>& v)
  {
    s << indent << "image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.image);
    s << indent << "custom_string: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.custom_string);
  }
};

} // namespace message_operations
} // namespace ros

#endif // YOLOINFER_MESSAGE_YOLOWITHSTRING_H