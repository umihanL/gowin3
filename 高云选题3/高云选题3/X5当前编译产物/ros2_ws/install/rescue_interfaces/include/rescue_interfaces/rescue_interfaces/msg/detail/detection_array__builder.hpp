// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectionArray_detections
{
public:
  explicit Init_DetectionArray_detections(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::DetectionArray detections(::rescue_interfaces::msg::DetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_stereo_calibration_id
{
public:
  explicit Init_DetectionArray_stereo_calibration_id(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_detections stereo_calibration_id(::rescue_interfaces::msg::DetectionArray::_stereo_calibration_id_type arg)
  {
    msg_.stereo_calibration_id = std::move(arg);
    return Init_DetectionArray_detections(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_status
{
public:
  explicit Init_DetectionArray_status(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_stereo_calibration_id status(::rescue_interfaces::msg::DetectionArray::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_DetectionArray_stereo_calibration_id(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_detector_available
{
public:
  explicit Init_DetectionArray_detector_available(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_status detector_available(::rescue_interfaces::msg::DetectionArray::_detector_available_type arg)
  {
    msg_.detector_available = std::move(arg);
    return Init_DetectionArray_status(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_rectified
{
public:
  explicit Init_DetectionArray_rectified(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_detector_available rectified(::rescue_interfaces::msg::DetectionArray::_rectified_type arg)
  {
    msg_.rectified = std::move(arg);
    return Init_DetectionArray_detector_available(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_height
{
public:
  explicit Init_DetectionArray_height(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_rectified height(::rescue_interfaces::msg::DetectionArray::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_DetectionArray_rectified(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_width
{
public:
  explicit Init_DetectionArray_width(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_height width(::rescue_interfaces::msg::DetectionArray::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_DetectionArray_height(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_sequence
{
public:
  explicit Init_DetectionArray_sequence(::rescue_interfaces::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_width sequence(::rescue_interfaces::msg::DetectionArray::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return Init_DetectionArray_width(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

class Init_DetectionArray_header
{
public:
  Init_DetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionArray_sequence header(::rescue_interfaces::msg::DetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectionArray_sequence(msg_);
  }

private:
  ::rescue_interfaces::msg::DetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::DetectionArray>()
{
  return rescue_interfaces::msg::builder::Init_DetectionArray_header();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
