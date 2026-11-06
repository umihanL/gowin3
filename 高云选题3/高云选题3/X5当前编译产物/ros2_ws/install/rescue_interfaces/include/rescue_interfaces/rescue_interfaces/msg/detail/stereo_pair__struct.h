// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/StereoPair.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__STRUCT_H_

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
// Member 'left'
// Member 'right'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/StereoPair in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__StereoPair
{
  std_msgs__msg__Header header;
  uint64_t sequence;
  sensor_msgs__msg__Image left;
  sensor_msgs__msg__Image right;
  /// True only after exposure synchronization of this acquisition mode is verified.
  bool synchronization_verified;
  double skew_s;
} rescue_interfaces__msg__StereoPair;

// Struct for a sequence of rescue_interfaces__msg__StereoPair.
typedef struct rescue_interfaces__msg__StereoPair__Sequence
{
  rescue_interfaces__msg__StereoPair * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__StereoPair__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__STEREO_PAIR__STRUCT_H_
