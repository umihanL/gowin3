// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rescue_interfaces:msg/ManeuverCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__STRUCT_H_
#define RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FOLLOW_LANE'.
enum
{
  rescue_interfaces__msg__ManeuverCommand__FOLLOW_LANE = 0
};

/// Constant 'TURN_LEFT'.
enum
{
  rescue_interfaces__msg__ManeuverCommand__TURN_LEFT = 1
};

/// Constant 'TURN_RIGHT'.
enum
{
  rescue_interfaces__msg__ManeuverCommand__TURN_RIGHT = 2
};

/// Constant 'STRAIGHT'.
enum
{
  rescue_interfaces__msg__ManeuverCommand__STRAIGHT = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'command_id'
// Member 'edge_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ManeuverCommand in the package rescue_interfaces.
typedef struct rescue_interfaces__msg__ManeuverCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String command_id;
  rosidl_runtime_c__String edge_id;
  uint32_t route_revision;
  uint8_t turn;
  bool stop_at_end;
  float speed_limit_m_s;
  float distance_m;
  uint16_t lease_ms;
} rescue_interfaces__msg__ManeuverCommand;

// Struct for a sequence of rescue_interfaces__msg__ManeuverCommand.
typedef struct rescue_interfaces__msg__ManeuverCommand__Sequence
{
  rescue_interfaces__msg__ManeuverCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rescue_interfaces__msg__ManeuverCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__MANEUVER_COMMAND__STRUCT_H_
