// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rescue_interfaces:msg/ManeuverCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__TRAITS_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rescue_interfaces/msg/detail/maneuver_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rescue_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ManeuverCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: command_id
  {
    out << "command_id: ";
    rosidl_generator_traits::value_to_yaml(msg.command_id, out);
    out << ", ";
  }

  // member: edge_id
  {
    out << "edge_id: ";
    rosidl_generator_traits::value_to_yaml(msg.edge_id, out);
    out << ", ";
  }

  // member: route_revision
  {
    out << "route_revision: ";
    rosidl_generator_traits::value_to_yaml(msg.route_revision, out);
    out << ", ";
  }

  // member: turn
  {
    out << "turn: ";
    rosidl_generator_traits::value_to_yaml(msg.turn, out);
    out << ", ";
  }

  // member: stop_at_end
  {
    out << "stop_at_end: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_at_end, out);
    out << ", ";
  }

  // member: speed_limit_m_s
  {
    out << "speed_limit_m_s: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_limit_m_s, out);
    out << ", ";
  }

  // member: distance_m
  {
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << ", ";
  }

  // member: lease_ms
  {
    out << "lease_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_ms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManeuverCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: command_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_id: ";
    rosidl_generator_traits::value_to_yaml(msg.command_id, out);
    out << "\n";
  }

  // member: edge_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "edge_id: ";
    rosidl_generator_traits::value_to_yaml(msg.edge_id, out);
    out << "\n";
  }

  // member: route_revision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route_revision: ";
    rosidl_generator_traits::value_to_yaml(msg.route_revision, out);
    out << "\n";
  }

  // member: turn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "turn: ";
    rosidl_generator_traits::value_to_yaml(msg.turn, out);
    out << "\n";
  }

  // member: stop_at_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_at_end: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_at_end, out);
    out << "\n";
  }

  // member: speed_limit_m_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_limit_m_s: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_limit_m_s, out);
    out << "\n";
  }

  // member: distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << "\n";
  }

  // member: lease_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lease_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_ms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManeuverCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rescue_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rescue_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rescue_interfaces::msg::ManeuverCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  rescue_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rescue_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rescue_interfaces::msg::ManeuverCommand & msg)
{
  return rescue_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rescue_interfaces::msg::ManeuverCommand>()
{
  return "rescue_interfaces::msg::ManeuverCommand";
}

template<>
inline const char * name<rescue_interfaces::msg::ManeuverCommand>()
{
  return "rescue_interfaces/msg/ManeuverCommand";
}

template<>
struct has_fixed_size<rescue_interfaces::msg::ManeuverCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rescue_interfaces::msg::ManeuverCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rescue_interfaces::msg::ManeuverCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__TRAITS_HPP_
