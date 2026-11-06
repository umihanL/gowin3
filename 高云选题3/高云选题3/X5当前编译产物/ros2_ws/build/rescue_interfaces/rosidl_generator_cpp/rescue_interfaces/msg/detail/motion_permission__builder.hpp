// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/MotionPermission.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/motion_permission__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotionPermission_reason
{
public:
  explicit Init_MotionPermission_reason(::rescue_interfaces::msg::MotionPermission & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::MotionPermission reason(::rescue_interfaces::msg::MotionPermission::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::MotionPermission msg_;
};

class Init_MotionPermission_lease_ms
{
public:
  explicit Init_MotionPermission_lease_ms(::rescue_interfaces::msg::MotionPermission & msg)
  : msg_(msg)
  {}
  Init_MotionPermission_reason lease_ms(::rescue_interfaces::msg::MotionPermission::_lease_ms_type arg)
  {
    msg_.lease_ms = std::move(arg);
    return Init_MotionPermission_reason(msg_);
  }

private:
  ::rescue_interfaces::msg::MotionPermission msg_;
};

class Init_MotionPermission_allow_motion
{
public:
  explicit Init_MotionPermission_allow_motion(::rescue_interfaces::msg::MotionPermission & msg)
  : msg_(msg)
  {}
  Init_MotionPermission_lease_ms allow_motion(::rescue_interfaces::msg::MotionPermission::_allow_motion_type arg)
  {
    msg_.allow_motion = std::move(arg);
    return Init_MotionPermission_lease_ms(msg_);
  }

private:
  ::rescue_interfaces::msg::MotionPermission msg_;
};

class Init_MotionPermission_event_id
{
public:
  explicit Init_MotionPermission_event_id(::rescue_interfaces::msg::MotionPermission & msg)
  : msg_(msg)
  {}
  Init_MotionPermission_allow_motion event_id(::rescue_interfaces::msg::MotionPermission::_event_id_type arg)
  {
    msg_.event_id = std::move(arg);
    return Init_MotionPermission_allow_motion(msg_);
  }

private:
  ::rescue_interfaces::msg::MotionPermission msg_;
};

class Init_MotionPermission_header
{
public:
  Init_MotionPermission_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionPermission_event_id header(::rescue_interfaces::msg::MotionPermission::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotionPermission_event_id(msg_);
  }

private:
  ::rescue_interfaces::msg::MotionPermission msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::MotionPermission>()
{
  return rescue_interfaces::msg::builder::Init_MotionPermission_header();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__BUILDER_HPP_
