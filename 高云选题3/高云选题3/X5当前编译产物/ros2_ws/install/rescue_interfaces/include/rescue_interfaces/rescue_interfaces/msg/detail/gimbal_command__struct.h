// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'owner'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GimbalCommand in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__GimbalCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String owner;
  float pan_deg;
  float tilt_deg;
  float speed_deg_s;
  uint16_t lease_ms;
} rescue_interfaces__msg__GimbalCommand;

// Struct for a sequence of rescue_interfaces__msg__GimbalCommand.
typedef struct rescue_interfaces__msg__GimbalCommand__Sequence
{
  rescue_interfaces__msg__GimbalCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__GimbalCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_
