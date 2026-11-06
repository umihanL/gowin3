// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/fpga_event__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rescue_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rescue_interfaces/msg/detail/fpga_event__struct.h"
#include "rescue_interfaces/msg/detail/fpga_event__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rescue_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rescue_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rescue_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _FpgaEvent__ros_msg_type = rescue_interfaces__msg__FpgaEvent;

static bool _FpgaEvent__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FpgaEvent__ros_msg_type * ros_message = static_cast<const _FpgaEvent__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: event_type
  {
    cdr << ros_message->event_type;
  }

  // Field name: wire_sequence
  {
    cdr << ros_message->wire_sequence;
  }

  // Field name: event_id
  {
    cdr << ros_message->event_id;
  }

  // Field name: vehicle_stopped
  {
    cdr << (ros_message->vehicle_stopped ? true : false);
  }

  // Field name: gimbal_settled
  {
    cdr << (ros_message->gimbal_settled ? true : false);
  }

  // Field name: measured_angles_valid
  {
    cdr << (ros_message->measured_angles_valid ? true : false);
  }

  // Field name: pan_deg
  {
    cdr << ros_message->pan_deg;
  }

  // Field name: tilt_deg
  {
    cdr << ros_message->tilt_deg;
  }

  // Field name: acknowledged_sequence
  {
    cdr << ros_message->acknowledged_sequence;
  }

  // Field name: acknowledgement_result
  {
    cdr << ros_message->acknowledgement_result;
  }

  return true;
}

static bool _FpgaEvent__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FpgaEvent__ros_msg_type * ros_message = static_cast<_FpgaEvent__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: event_type
  {
    cdr >> ros_message->event_type;
  }

  // Field name: wire_sequence
  {
    cdr >> ros_message->wire_sequence;
  }

  // Field name: event_id
  {
    cdr >> ros_message->event_id;
  }

  // Field name: vehicle_stopped
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->vehicle_stopped = tmp ? true : false;
  }

  // Field name: gimbal_settled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->gimbal_settled = tmp ? true : false;
  }

  // Field name: measured_angles_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->measured_angles_valid = tmp ? true : false;
  }

  // Field name: pan_deg
  {
    cdr >> ros_message->pan_deg;
  }

  // Field name: tilt_deg
  {
    cdr >> ros_message->tilt_deg;
  }

  // Field name: acknowledged_sequence
  {
    cdr >> ros_message->acknowledged_sequence;
  }

  // Field name: acknowledgement_result
  {
    cdr >> ros_message->acknowledgement_result;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rescue_interfaces
size_t get_serialized_size_rescue_interfaces__msg__FpgaEvent(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FpgaEvent__ros_msg_type * ros_message = static_cast<const _FpgaEvent__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name event_type
  {
    size_t item_size = sizeof(ros_message->event_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wire_sequence
  {
    size_t item_size = sizeof(ros_message->wire_sequence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name event_id
  {
    size_t item_size = sizeof(ros_message->event_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vehicle_stopped
  {
    size_t item_size = sizeof(ros_message->vehicle_stopped);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gimbal_settled
  {
    size_t item_size = sizeof(ros_message->gimbal_settled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name measured_angles_valid
  {
    size_t item_size = sizeof(ros_message->measured_angles_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pan_deg
  {
    size_t item_size = sizeof(ros_message->pan_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tilt_deg
  {
    size_t item_size = sizeof(ros_message->tilt_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acknowledged_sequence
  {
    size_t item_size = sizeof(ros_message->acknowledged_sequence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acknowledgement_result
  {
    size_t item_size = sizeof(ros_message->acknowledgement_result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FpgaEvent__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rescue_interfaces__msg__FpgaEvent(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rescue_interfaces
size_t max_serialized_size_rescue_interfaces__msg__FpgaEvent(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: event_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: wire_sequence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: event_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vehicle_stopped
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: gimbal_settled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: measured_angles_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pan_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tilt_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: acknowledged_sequence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: acknowledgement_result
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rescue_interfaces__msg__FpgaEvent;
    is_plain =
      (
      offsetof(DataType, acknowledgement_result) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _FpgaEvent__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rescue_interfaces__msg__FpgaEvent(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FpgaEvent = {
  "rescue_interfaces::msg",
  "FpgaEvent",
  _FpgaEvent__cdr_serialize,
  _FpgaEvent__cdr_deserialize,
  _FpgaEvent__get_serialized_size,
  _FpgaEvent__max_serialized_size
};

static rosidl_message_type_support_t _FpgaEvent__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FpgaEvent,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rescue_interfaces, msg, FpgaEvent)() {
  return &_FpgaEvent__type_support;
}

#if defined(__cplusplus)
}
#endif
