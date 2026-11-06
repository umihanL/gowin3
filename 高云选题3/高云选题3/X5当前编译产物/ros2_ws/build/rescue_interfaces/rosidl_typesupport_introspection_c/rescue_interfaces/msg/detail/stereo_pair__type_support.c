// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rescue_interfaces:msg/StereoPair.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rescue_interfaces/msg/detail/stereo_pair__rosidl_typesupport_introspection_c.h"
#include "rescue_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rescue_interfaces/msg/detail/stereo_pair__functions.h"
#include "rescue_interfaces/msg/detail/stereo_pair__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `left`
// Member `right`
#include "sensor_msgs/msg/image.h"
// Member `left`
// Member `right`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rescue_interfaces__msg__StereoPair__init(message_memory);
}

void rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_fini_function(void * message_memory)
{
  rescue_interfaces__msg__StereoPair__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__StereoPair, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sequence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__StereoPair, sequence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__StereoPair, left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__StereoPair, right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "synchronization_verified",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__StereoPair, synchronization_verified),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "skew_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rescue_interfaces__msg__StereoPair, skew_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_members = {
  "rescue_interfaces__msg",  // message namespace
  "StereoPair",  // message name
  6,  // number of fields
  sizeof(rescue_interfaces__msg__StereoPair),
  rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_member_array,  // message members
  rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_init_function,  // function to initialize message memory (memory has to be allocated)
  rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_type_support_handle = {
  0,
  &rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rescue_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rescue_interfaces, msg, StereoPair)() {
  rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_type_support_handle.typesupport_identifier) {
    rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rescue_interfaces__msg__StereoPair__rosidl_typesupport_introspection_c__StereoPair_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
