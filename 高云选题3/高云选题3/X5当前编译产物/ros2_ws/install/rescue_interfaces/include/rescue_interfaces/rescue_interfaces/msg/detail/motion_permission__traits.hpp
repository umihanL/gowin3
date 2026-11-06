// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rescue_interfaces:msg/MotionPermission.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__TRAITS_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rescue_interfaces/msg/detail/motion_permission__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rescue_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotionPermission & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: event_id
  {
    out << "event_id: ";
    rosidl_generator_traits::value_to_yaml(msg.event_id, out);
    out << ", ";
  }

  // member: allow_motion
  {
    out << "allow_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_motion, out);
    out << ", ";
  }

  // member: lease_ms
  {
    out << "lease_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_ms, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotionPermission & msg,
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

  // member: event_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event_id: ";
    rosidl_generator_traits::value_to_yaml(msg.event_id, out);
    out << "\n";
  }

  // member: allow_motion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "allow_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_motion, out);
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

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotionPermission & msg, bool use_flow_style = false)
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
  const rescue_interfaces::msg::MotionPermission & msg,
  std::ostream & out, size_t indentation = 0)
{
  rescue_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rescue_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rescue_interfaces::msg::MotionPermission & msg)
{
  return rescue_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rescue_interfaces::msg::MotionPermission>()
{
  return "rescue_interfaces::msg::MotionPermission";
}

template<>
inline const char * name<rescue_interfaces::msg::MotionPermission>()
{
  return "rescue_interfaces/msg/MotionPermission";
}

template<>
struct has_fixed_size<rescue_interfaces::msg::MotionPermission>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rescue_interfaces::msg::MotionPermission>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rescue_interfaces::msg::MotionPermission>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__TRAITS_HPP_
