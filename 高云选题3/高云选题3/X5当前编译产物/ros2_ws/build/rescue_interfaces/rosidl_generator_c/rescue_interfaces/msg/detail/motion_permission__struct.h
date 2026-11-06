// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/MotionPermission.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__STRUCT_H_

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
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotionPermission in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__MotionPermission
{
  std_msgs__msg__Header header;
  uint32_t event_id;
  bool allow_motion;
  uint16_t lease_ms;
  rosidl_runtime_c__String reason;
} rescue_interfaces__msg__MotionPermission;

// Struct for a sequence of rescue_interfaces__msg__MotionPermission.
typedef struct rescue_interfaces__msg__MotionPermission__Sequence
{
  rescue_interfaces__msg__MotionPermission * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__MotionPermission__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MOTION_PERMISSION__STRUCT_H_
