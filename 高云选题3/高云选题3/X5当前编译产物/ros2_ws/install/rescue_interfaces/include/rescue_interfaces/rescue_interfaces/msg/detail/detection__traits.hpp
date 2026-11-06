// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rescue_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rescue_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position_m'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rescue_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << ", ";
  }

  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: bbox_xyxy
  {
    if (msg.bbox_xyxy.size() == 0) {
      out << "bbox_xyxy: []";
    } else {
      out << "bbox_xyxy: [";
      size_t pending_items = msg.bbox_xyxy.size();
      for (auto item : msg.bbox_xyxy) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: depth_valid
  {
    out << "depth_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_valid, out);
    out << ", ";
  }

  // member: position_m
  {
    out << "position_m: ";
    to_flow_style_yaml(msg.position_m, out);
    out << ", ";
  }

  // member: depth_m
  {
    out << "depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_m, out);
    out << ", ";
  }

  // member: depth_reason
  {
    out << "depth_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }

  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: bbox_xyxy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bbox_xyxy.size() == 0) {
      out << "bbox_xyxy: []\n";
    } else {
      out << "bbox_xyxy:\n";
      for (auto item : msg.bbox_xyxy) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: depth_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_valid, out);
    out << "\n";
  }

  // member: position_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_m:\n";
    to_block_style_yaml(msg.position_m, out, indentation + 2);
  }

  // member: depth_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_m: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_m, out);
    out << "\n";
  }

  // member: depth_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
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
  const rescue_interfaces::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  rescue_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rescue_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rescue_interfaces::msg::Detection & msg)
{
  return rescue_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rescue_interfaces::msg::Detection>()
{
  return "rescue_interfaces::msg::Detection";
}

template<>
inline const char * name<rescue_interfaces::msg::Detection>()
{
  return "rescue_interfaces/msg/Detection";
}

template<>
struct has_fixed_size<rescue_interfaces::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rescue_interfaces::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rescue_interfaces::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_
