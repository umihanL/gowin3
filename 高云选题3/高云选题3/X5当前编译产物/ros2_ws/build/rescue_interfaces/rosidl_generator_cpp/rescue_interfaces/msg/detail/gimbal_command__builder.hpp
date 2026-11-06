// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/gimbal_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_GimbalCommand_lease_ms
{
public:
  explicit Init_GimbalCommand_lease_ms(::rescue_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::GimbalCommand lease_ms(::rescue_interfaces::msg::GimbalCommand::_lease_ms_type arg)
  {
    msg_.lease_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_speed_deg_s
{
public:
  explicit Init_GimbalCommand_speed_deg_s(::rescue_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  Init_GimbalCommand_lease_ms speed_deg_s(::rescue_interfaces::msg::GimbalCommand::_speed_deg_s_type arg)
  {
    msg_.speed_deg_s = std::move(arg);
    return Init_GimbalCommand_lease_ms(msg_);
  }

private:
  ::rescue_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_tilt_deg
{
public:
  explicit Init_GimbalCommand_tilt_deg(::rescue_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  Init_GimbalCommand_speed_deg_s tilt_deg(::rescue_interfaces::msg::GimbalCommand::_tilt_deg_type arg)
  {
    msg_.tilt_deg = std::move(arg);
    return Init_GimbalCommand_speed_deg_s(msg_);
  }

private:
  ::rescue_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_pan_deg
{
public:
  explicit Init_GimbalCommand_pan_deg(::rescue_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  Init_GimbalCommand_tilt_deg pan_deg(::rescue_interfaces::msg::GimbalCommand::_pan_deg_type arg)
  {
    msg_.pan_deg = std::move(arg);
    return Init_GimbalCommand_tilt_deg(msg_);
  }

private:
  ::rescue_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_owner
{
public:
  explicit Init_GimbalCommand_owner(::rescue_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  Init_GimbalCommand_pan_deg owner(::rescue_interfaces::msg::GimbalCommand::_owner_type arg)
  {
    msg_.owner = std::move(arg);
    return Init_GimbalCommand_pan_deg(msg_);
  }

private:
  ::rescue_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_header
{
public:
  Init_GimbalCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalCommand_owner header(::rescue_interfaces::msg::GimbalCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalCommand_owner(msg_);
  }

private:
  ::rescue_interfaces::msg::GimbalCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::GimbalCommand>()
{
  return rescue_interfaces::msg::builder::Init_GimbalCommand_header();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_
