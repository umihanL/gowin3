// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/ManeuverCommand.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/maneuver_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `command_id`
// Member `edge_id`
#include "rosidl_runtime_c/string_functions.h"

bool
rescue_interfaces__msg__ManeuverCommand__init(rescue_interfaces__msg__ManeuverCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rescue_interfaces__msg__ManeuverCommand__fini(msg);
    return false;
  }
  // command_id
  if (!rosidl_runtime_c__String__init(&msg->command_id)) {
    rescue_interfaces__msg__ManeuverCommand__fini(msg);
    return false;
  }
  // edge_id
  if (!rosidl_runtime_c__String__init(&msg->edge_id)) {
    rescue_interfaces__msg__ManeuverCommand__fini(msg);
    return false;
  }
  // route_revision
  // turn
  // stop_at_end
  // speed_limit_m_s
  // distance_m
  // lease_ms
  return true;
}

void
rescue_interfaces__msg__ManeuverCommand__fini(rescue_interfaces__msg__ManeuverCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // command_id
  rosidl_runtime_c__String__fini(&msg->command_id);
  // edge_id
  rosidl_runtime_c__String__fini(&msg->edge_id);
  // route_revision
  // turn
  // stop_at_end
  // speed_limit_m_s
  // distance_m
  // lease_ms
}

bool
rescue_interfaces__msg__ManeuverCommand__are_equal(const rescue_interfaces__msg__ManeuverCommand * lhs, const rescue_interfaces__msg__ManeuverCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // command_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command_id), &(rhs->command_id)))
  {
    return false;
  }
  // edge_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->edge_id), &(rhs->edge_id)))
  {
    return false;
  }
  // route_revision
  if (lhs->route_revision != rhs->route_revision) {
    return false;
  }
  // turn
  if (lhs->turn != rhs->turn) {
    return false;
  }
  // stop_at_end
  if (lhs->stop_at_end != rhs->stop_at_end) {
    return false;
  }
  // speed_limit_m_s
  if (lhs->speed_limit_m_s != rhs->speed_limit_m_s) {
    return false;
  }
  // distance_m
  if (lhs->distance_m != rhs->distance_m) {
    return false;
  }
  // lease_ms
  if (lhs->lease_ms != rhs->lease_ms) {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__ManeuverCommand__copy(
  const rescue_interfaces__msg__ManeuverCommand * input,
  rescue_interfaces__msg__ManeuverCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // command_id
  if (!rosidl_runtime_c__String__copy(
      &(input->command_id), &(output->command_id)))
  {
    return false;
  }
  // edge_id
  if (!rosidl_runtime_c__String__copy(
      &(input->edge_id), &(output->edge_id)))
  {
    return false;
  }
  // route_revision
  output->route_revision = input->route_revision;
  // turn
  output->turn = input->turn;
  // stop_at_end
  output->stop_at_end = input->stop_at_end;
  // speed_limit_m_s
  output->speed_limit_m_s = input->speed_limit_m_s;
  // distance_m
  output->distance_m = input->distance_m;
  // lease_ms
  output->lease_ms = input->lease_ms;
  return true;
}

rescue_interfaces__msg__ManeuverCommand *
rescue_interfaces__msg__ManeuverCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__ManeuverCommand * msg = (rescue_interfaces__msg__ManeuverCommand *)allocator.allocate(sizeof(rescue_interfaces__msg__ManeuverCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__ManeuverCommand));
  bool success = rescue_interfaces__msg__ManeuverCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__ManeuverCommand__destroy(rescue_interfaces__msg__ManeuverCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__ManeuverCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__ManeuverCommand__Sequence__init(rescue_interfaces__msg__ManeuverCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__ManeuverCommand * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__ManeuverCommand *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__ManeuverCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__ManeuverCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__ManeuverCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rescue_interfaces__msg__ManeuverCommand__Sequence__fini(rescue_interfaces__msg__ManeuverCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rescue_interfaces__msg__ManeuverCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rescue_interfaces__msg__ManeuverCommand__Sequence *
rescue_interfaces__msg__ManeuverCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__ManeuverCommand__Sequence * array = (rescue_interfaces__msg__ManeuverCommand__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__ManeuverCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__ManeuverCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__ManeuverCommand__Sequence__destroy(rescue_interfaces__msg__ManeuverCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__ManeuverCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__ManeuverCommand__Sequence__are_equal(const rescue_interfaces__msg__ManeuverCommand__Sequence * lhs, const rescue_interfaces__msg__ManeuverCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__ManeuverCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__ManeuverCommand__Sequence__copy(
  const rescue_interfaces__msg__ManeuverCommand__Sequence * input,
  rescue_interfaces__msg__ManeuverCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__ManeuverCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__ManeuverCommand * data =
      (rescue_interfaces__msg__ManeuverCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__ManeuverCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__ManeuverCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__ManeuverCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
