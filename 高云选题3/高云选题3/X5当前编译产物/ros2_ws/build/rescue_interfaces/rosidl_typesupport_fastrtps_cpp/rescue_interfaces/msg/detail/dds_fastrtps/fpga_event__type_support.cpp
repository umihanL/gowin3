// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/fpga_event__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rescue_interfaces/msg/detail/fpga_event__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace rescue_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rescue_interfaces
cdr_serialize(
  const rescue_interfaces::msg::FpgaEvent & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: event_type
  cdr << ros_message.event_type;
  // Member: wire_sequence
  cdr << ros_message.wire_sequence;
  // Member: event_id
  cdr << ros_message.event_id;
  // Member: vehicle_stopped
  cdr << (ros_message.vehicle_stopped ? true : false);
  // Member: gimbal_settled
  cdr << (ros_message.gimbal_settled ? true : false);
  // Member: measured_angles_valid
  cdr << (ros_message.measured_angles_valid ? true : false);
  // Member: pan_deg
  cdr << ros_message.pan_deg;
  // Member: tilt_deg
  cdr << ros_message.tilt_deg;
  // Member: acknowledged_sequence
  cdr << ros_message.acknowledged_sequence;
  // Member: acknowledgement_result
  cdr << ros_message.acknowledgement_result;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rescue_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rescue_interfaces::msg::FpgaEvent & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: event_type
  cdr >> ros_message.event_type;

  // Member: wire_sequence
  cdr >> ros_message.wire_sequence;

  // Member: event_id
  cdr >> ros_message.event_id;

  // Member: vehicle_stopped
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.vehicle_stopped = tmp ? true : false;
  }

  // Member: gimbal_settled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.gimbal_settled = tmp ? true : false;
  }

  // Member: measured_angles_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.measured_angles_valid = tmp ? true : false;
  }

  // Member: pan_deg
  cdr >> ros_message.pan_deg;

  // Member: tilt_deg
  cdr >> ros_message.tilt_deg;

  // Member: acknowledged_sequence
  cdr >> ros_message.acknowledged_sequence;

  // Member: acknowledgement_result
  cdr >> ros_message.acknowledgement_result;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rescue_interfaces
get_serialized_size(
  const rescue_interfaces::msg::FpgaEvent & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: event_type
  {
    size_t item_size = sizeof(ros_message.event_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wire_sequence
  {
    size_t item_size = sizeof(ros_message.wire_sequence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: event_id
  {
    size_t item_size = sizeof(ros_message.event_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vehicle_stopped
  {
    size_t item_size = sizeof(ros_message.vehicle_stopped);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gimbal_settled
  {
    size_t item_size = sizeof(ros_message.gimbal_settled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measured_angles_valid
  {
    size_t item_size = sizeof(ros_message.measured_angles_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pan_deg
  {
    size_t item_size = sizeof(ros_message.pan_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tilt_deg
  {
    size_t item_size = sizeof(ros_message.tilt_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acknowledged_sequence
  {
    size_t item_size = sizeof(ros_message.acknowledged_sequence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acknowledgement_result
  {
    size_t item_size = sizeof(ros_message.acknowledgement_result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rescue_interfaces
max_serialized_size_FpgaEvent(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: event_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: wire_sequence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: event_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vehicle_stopped
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gimbal_settled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: measured_angles_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pan_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tilt_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: acknowledged_sequence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: acknowledgement_result
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
    using DataType = rescue_interfaces::msg::FpgaEvent;
    is_plain =
      (
      offsetof(DataType, acknowledgement_result) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _FpgaEvent__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rescue_interfaces::msg::FpgaEvent *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FpgaEvent__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rescue_interfaces::msg::FpgaEvent *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FpgaEvent__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rescue_interfaces::msg::FpgaEvent *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FpgaEvent__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FpgaEvent(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FpgaEvent__callbacks = {
  "rescue_interfaces::msg",
  "FpgaEvent",
  _FpgaEvent__cdr_serialize,
  _FpgaEvent__cdr_deserialize,
  _FpgaEvent__get_serialized_size,
  _FpgaEvent__max_serialized_size
};

static rosidl_message_type_support_t _FpgaEvent__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FpgaEvent__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rescue_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rescue_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<rescue_interfaces::msg::FpgaEvent>()
{
  return &rescue_interfaces::msg::typesupport_fastrtps_cpp::_FpgaEvent__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rescue_interfaces, msg, FpgaEvent)() {
  return &rescue_interfaces::msg::typesupport_fastrtps_cpp::_FpgaEvent__handle;
}

#ifdef __cplusplus
}
#endif
