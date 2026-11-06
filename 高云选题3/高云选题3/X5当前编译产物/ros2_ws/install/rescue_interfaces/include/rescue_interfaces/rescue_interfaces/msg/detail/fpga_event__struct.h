// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CROSSWALK'.
enum
{
  rescue_interfaces__msg__FpgaEvent__CROSSWALK = 16
};

/// Constant 'CROSSING_COMPLETE'.
enum
{
  rescue_interfaces__msg__FpgaEvent__CROSSING_COMPLETE = 17
};

/// Constant 'STATUS'.
enum
{
  rescue_interfaces__msg__FpgaEvent__STATUS = 18
};

/// Constant 'ACK'.
enum
{
  rescue_interfaces__msg__FpgaEvent__ACK = 19
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/FpgaEvent in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__FpgaEvent
{
  std_msgs__msg__Header header;
  uint8_t event_type;
  uint16_t wire_sequence;
  uint32_t event_id;
  bool vehicle_stopped;
  bool gimbal_settled;
  bool measured_angles_valid;
  float pan_deg;
  float tilt_deg;
  uint16_t acknowledged_sequence;
  uint8_t acknowledgement_result;
} rescue_interfaces__msg__FpgaEvent;

// Struct for a sequence of rescue_interfaces__msg__FpgaEvent.
typedef struct rescue_interfaces__msg__FpgaEvent__Sequence
{
  rescue_interfaces__msg__FpgaEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__FpgaEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__FPGA_EVENT__STRUCT_H_
