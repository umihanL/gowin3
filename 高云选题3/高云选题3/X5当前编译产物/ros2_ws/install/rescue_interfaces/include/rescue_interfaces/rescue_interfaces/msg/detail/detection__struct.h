// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
// Member 'depth_reason'
#include "rosidl_runtime_c/string.h"
// Member 'position_m'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Detection in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__Detection
{
  int32_t class_id;
  rosidl_runtime_c__String class_name;
  float confidence;
  /// Original selected-eye pixels; rectified if DetectionArray.rectified is true.
  float bbox_xyxy[4];
  bool depth_valid;
  geometry_msgs__msg__Point position_m;
  float depth_m;
  rosidl_runtime_c__String depth_reason;
} rescue_interfaces__msg__Detection;

// Struct for a sequence of rescue_interfaces__msg__Detection.
typedef struct rescue_interfaces__msg__Detection__Sequence
{
  rescue_interfaces__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
