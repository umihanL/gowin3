// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rescue_interfaces/msg/detail/gimbal_command__rosidl_typesupport_introspection_c.h"
#include "rescue_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rescue_interfaces/msg/detail/gimbal_command__functions.h"
#include "rescue_interfaces/msg/detail/gimbal_command__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `owner`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rescue_interfaces__msg__GimbalCommand__init(message_memory);
}

void rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_fini_function(void * message_memory)
{
  rescue_interfaces__msg__GimbalCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__GimbalCommand, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "owner",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__GimbalCommand, owner),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pan_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__GimbalCommand, pan_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tilt_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__GimbalCommand, tilt_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_deg_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__GimbalCommand, speed_deg_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lease_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__GimbalCommand, lease_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_members = {
  "rescue_interfaces__msg",  // message namespace
  "GimbalCommand",  // message name
  6,  // number of fields
  sizeof(rescue_interfaces__msg__GimbalCommand),
  rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_member_array,  // message members
  rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_type_support_handle = {
  0,
  &rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rescue_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rescue_interfaces, msg, GimbalCommand)() {
  rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_type_support_handle.typesupport_identifier) {
    rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rescue_interfaces__msg__GimbalCommand__rosidl_typesupport_introspection_c__GimbalCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
