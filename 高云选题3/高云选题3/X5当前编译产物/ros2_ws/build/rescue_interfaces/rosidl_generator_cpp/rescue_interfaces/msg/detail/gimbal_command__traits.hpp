// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rescue_interfaces/msg/detail/gimbal_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rescue_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: owner
  {
    out << "owner: ";
    rosidl_generator_traits::value_to_yaml(msg.owner, out);
    out << ", ";
  }

  // member: pan_deg
  {
    out << "pan_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_deg, out);
    out << ", ";
  }

  // member: tilt_deg
  {
    out << "tilt_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_deg, out);
    out << ", ";
  }

  // member: speed_deg_s
  {
    out << "speed_deg_s: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_deg_s, out);
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
  const GimbalCommand & msg,
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

  // member: owner
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "owner: ";
    rosidl_generator_traits::value_to_yaml(msg.owner, out);
    out << "\n";
  }

  // member: pan_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pan_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pan_deg, out);
    out << "\n";
  }

  // member: tilt_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tilt_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_deg, out);
    out << "\n";
  }

  // member: speed_deg_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_deg_s: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_deg_s, out);
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

inline std::string to_yaml(const GimbalCommand & msg, bool use_flow_style = false)
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
  const rescue_interfaces::msg::GimbalCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  rescue_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rescue_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rescue_interfaces::msg::GimbalCommand & msg)
{
  return rescue_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rescue_interfaces::msg::GimbalCommand>()
{
  return "rescue_interfaces::msg::GimbalCommand";
}

template<>
inline const char * name<rescue_interfaces::msg::GimbalCommand>()
{
  return "rescue_interfaces/msg/GimbalCommand";
}

template<>
struct has_fixed_size<rescue_interfaces::msg::GimbalCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rescue_interfaces::msg::GimbalCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rescue_interfaces::msg::GimbalCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_
