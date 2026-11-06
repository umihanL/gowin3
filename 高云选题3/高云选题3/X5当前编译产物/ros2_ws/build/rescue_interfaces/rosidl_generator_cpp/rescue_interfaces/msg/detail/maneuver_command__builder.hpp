// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rescue_interfaces:msg/ManeuverCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__BUILDER_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rescue_interfaces/msg/detail/maneuver_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rescue_interfaces
{

namespace msg
{

namespace builder
{

class Init_ManeuverCommand_lease_ms
{
public:
  explicit Init_ManeuverCommand_lease_ms(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  ::rescue_interfaces::msg::ManeuverCommand lease_ms(::rescue_interfaces::msg::ManeuverCommand::_lease_ms_type arg)
  {
    msg_.lease_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_distance_m
{
public:
  explicit Init_ManeuverCommand_distance_m(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_lease_ms distance_m(::rescue_interfaces::msg::ManeuverCommand::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return Init_ManeuverCommand_lease_ms(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_speed_limit_m_s
{
public:
  explicit Init_ManeuverCommand_speed_limit_m_s(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_distance_m speed_limit_m_s(::rescue_interfaces::msg::ManeuverCommand::_speed_limit_m_s_type arg)
  {
    msg_.speed_limit_m_s = std::move(arg);
    return Init_ManeuverCommand_distance_m(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_stop_at_end
{
public:
  explicit Init_ManeuverCommand_stop_at_end(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_speed_limit_m_s stop_at_end(::rescue_interfaces::msg::ManeuverCommand::_stop_at_end_type arg)
  {
    msg_.stop_at_end = std::move(arg);
    return Init_ManeuverCommand_speed_limit_m_s(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_turn
{
public:
  explicit Init_ManeuverCommand_turn(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_stop_at_end turn(::rescue_interfaces::msg::ManeuverCommand::_turn_type arg)
  {
    msg_.turn = std::move(arg);
    return Init_ManeuverCommand_stop_at_end(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_route_revision
{
public:
  explicit Init_ManeuverCommand_route_revision(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_turn route_revision(::rescue_interfaces::msg::ManeuverCommand::_route_revision_type arg)
  {
    msg_.route_revision = std::move(arg);
    return Init_ManeuverCommand_turn(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_edge_id
{
public:
  explicit Init_ManeuverCommand_edge_id(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_route_revision edge_id(::rescue_interfaces::msg::ManeuverCommand::_edge_id_type arg)
  {
    msg_.edge_id = std::move(arg);
    return Init_ManeuverCommand_route_revision(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_command_id
{
public:
  explicit Init_ManeuverCommand_command_id(::rescue_interfaces::msg::ManeuverCommand & msg)
  : msg_(msg)
  {}
  Init_ManeuverCommand_edge_id command_id(::rescue_interfaces::msg::ManeuverCommand::_command_id_type arg)
  {
    msg_.command_id = std::move(arg);
    return Init_ManeuverCommand_edge_id(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

class Init_ManeuverCommand_header
{
public:
  Init_ManeuverCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManeuverCommand_command_id header(::rescue_interfaces::msg::ManeuverCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ManeuverCommand_command_id(msg_);
  }

private:
  ::rescue_interfaces::msg::ManeuverCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rescue_interfaces::msg::ManeuverCommand>()
{
  return rescue_interfaces::msg::builder::Init_ManeuverCommand_header();
}

}  // namespace rescue_interfaces

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__BUILDER_HPP_
