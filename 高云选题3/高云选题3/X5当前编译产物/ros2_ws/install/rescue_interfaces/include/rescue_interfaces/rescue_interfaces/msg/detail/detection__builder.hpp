// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_Detection_depth_reason
{
public:
  explicit Init_Detection_depth_reason(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::Detection depth_reason(::rescue_interfaces::msg::Detection::_depth_reason_type arg)
  {
    msg_.depth_reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_depth_m
{
public:
  explicit Init_Detection_depth_m(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_depth_reason depth_m(::rescue_interfaces::msg::Detection::_depth_m_type arg)
  {
    msg_.depth_m = std::move(arg);
    return Init_Detection_depth_reason(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_position_m
{
public:
  explicit Init_Detection_position_m(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_depth_m position_m(::rescue_interfaces::msg::Detection::_position_m_type arg)
  {
    msg_.position_m = std::move(arg);
    return Init_Detection_depth_m(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_depth_valid
{
public:
  explicit Init_Detection_depth_valid(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_position_m depth_valid(::rescue_interfaces::msg::Detection::_depth_valid_type arg)
  {
    msg_.depth_valid = std::move(arg);
    return Init_Detection_position_m(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_bbox_xyxy
{
public:
  explicit Init_Detection_bbox_xyxy(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_depth_valid bbox_xyxy(::rescue_interfaces::msg::Detection::_bbox_xyxy_type arg)
  {
    msg_.bbox_xyxy = std::move(arg);
    return Init_Detection_depth_valid(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_xyxy confidence(::rescue_interfaces::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_bbox_xyxy(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_class_name
{
public:
  explicit Init_Detection_class_name(::rescue_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_confidence class_name(::rescue_interfaces::msg::Detection::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

class Init_Detection_class_id
{
public:
  Init_Detection_class_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_class_name class_id(::rescue_interfaces::msg::Detection::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_Detection_class_name(msg_);
  }

private:
  ::rescue_interfaces::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::Detection>()
{
  return rescue_interfaces::msg::builder::Init_Detection_class_id();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
