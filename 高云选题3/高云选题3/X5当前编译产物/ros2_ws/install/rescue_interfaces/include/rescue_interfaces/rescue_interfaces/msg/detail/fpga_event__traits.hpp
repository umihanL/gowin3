// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__TRAITS_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rescue_interfaces/msg/detail/fpga_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rescue_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FpgaEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: event_type
  {
    out << "event_type: ";
    rosidl_generator_traits::value_to_yaml(msg.event_type, out);
    out << ", ";
  }

  // member: wire_sequence
  {
    out << "wire_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.wire_sequence, out);
    out << ", ";
  }

  // member: event_id
  {
    out << "event_id: ";
    rosidl_generator_traits::value_to_yaml(msg.event_id, out);
    out << ", ";
  }

  // member: vehicle_stopped
  {
    out << "vehicle_stopped: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_stopped, out);
    out << ", ";
  }

  // member: gimbal_settled
  {
    out << "gimbal_settled: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_settled, out);
    out << ", ";
  }

  // member: measured_angles_valid
  {
    out << "measured_angles_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_angles_valid, out);
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

  // member: acknowledged_sequence
  {
    out << "acknowledged_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledged_sequence, out);
    out << ", ";
  }

  // member: acknowledgement_result
  {
    out << "acknowledgement_result: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledgement_result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FpgaEvent & msg,
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

  // member: event_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event_type: ";
    rosidl_generator_traits::value_to_yaml(msg.event_type, out);
    out << "\n";
  }

  // member: wire_sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wire_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.wire_sequence, out);
    out << "\n";
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

  // member: vehicle_stopped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_stopped: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_stopped, out);
    out << "\n";
  }

  // member: gimbal_settled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_settled: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_settled, out);
    out << "\n";
  }

  // member: measured_angles_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_angles_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_angles_valid, out);
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

  // member: acknowledged_sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acknowledged_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledged_sequence, out);
    out << "\n";
  }

  // member: acknowledgement_result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acknowledgement_result: ";
    rosidl_generator_traits::value_to_yaml(msg.acknowledgement_result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FpgaEvent & msg, bool use_flow_style = false)
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
  const rescue_interfaces::msg::FpgaEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  rescue_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rescue_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rescue_interfaces::msg::FpgaEvent & msg)
{
  return rescue_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rescue_interfaces::msg::FpgaEvent>()
{
  return "rescue_interfaces::msg::FpgaEvent";
}

template<>
inline const char * name<rescue_interfaces::msg::FpgaEvent>()
{
  return "rescue_interfaces/msg/FpgaEvent";
}

template<>
struct has_fixed_size<rescue_interfaces::msg::FpgaEvent>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rescue_interfaces::msg::FpgaEvent>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rescue_interfaces::msg::FpgaEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__TRAITS_HPP_
