// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_

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
// Member 'status'
// Member 'stereo_calibration_id'
#include "rosidl_runtime_c/string.h"
// Member 'detections'
#include "rescue_interfaces/msg/detail/detection__struct.h"

/// Struct defined in msg/DetectionArray in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__DetectionArray
{
  std_msgs__msg__Header header;
  uint64_t sequence;
  uint32_t width;
  uint32_t height;
  bool rectified;
  bool detector_available;
  rosidl_runtime_c__String status;
  /// Empty unless a verified stereo depth producer enriched this exact exposure.
  rosidl_runtime_c__String stereo_calibration_id;
  rescue_interfaces__msg__Detection__Sequence detections;
} rescue_interfaces__msg__DetectionArray;

// Struct for a sequence of rescue_interfaces__msg__DetectionArray.
typedef struct rescue_interfaces__msg__DetectionArray__Sequence
{
  rescue_interfaces__msg__DetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__DetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
