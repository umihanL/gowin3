// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/MotionPermission.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/motion_permission__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
rescue_interfaces__msg__MotionPermission__init(rescue_interfaces__msg__MotionPermission * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rescue_interfaces__msg__MotionPermission__fini(msg);
    return false;
  }
  // event_id
  // allow_motion
  // lease_ms
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    rescue_interfaces__msg__MotionPermission__fini(msg);
    return false;
  }
  return true;
}

void
rescue_interfaces__msg__MotionPermission__fini(rescue_interfaces__msg__MotionPermission * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // event_id
  // allow_motion
  // lease_ms
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
rescue_interfaces__msg__MotionPermission__are_equal(const rescue_interfaces__msg__MotionPermission * lhs, const rescue_interfaces__msg__MotionPermission * rhs)
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
  // event_id
  if (lhs->event_id != rhs->event_id) {
    return false;
  }
  // allow_motion
  if (lhs->allow_motion != rhs->allow_motion) {
    return false;
  }
  // lease_ms
  if (lhs->lease_ms != rhs->lease_ms) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__MotionPermission__copy(
  const rescue_interfaces__msg__MotionPermission * input,
  rescue_interfaces__msg__MotionPermission * output)
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
  // event_id
  output->event_id = input->event_id;
  // allow_motion
  output->allow_motion = input->allow_motion;
  // lease_ms
  output->lease_ms = input->lease_ms;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

rescue_interfaces__msg__MotionPermission *
rescue_interfaces__msg__MotionPermission__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__MotionPermission * msg = (rescue_interfaces__msg__MotionPermission *)allocator.allocate(sizeof(rescue_interfaces__msg__MotionPermission), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__MotionPermission));
  bool success = rescue_interfaces__msg__MotionPermission__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__MotionPermission__destroy(rescue_interfaces__msg__MotionPermission * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__MotionPermission__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__MotionPermission__Sequence__init(rescue_interfaces__msg__MotionPermission__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__MotionPermission * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__MotionPermission *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__MotionPermission), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__MotionPermission__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__MotionPermission__fini(&data[i - 1]);
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
rescue_interfaces__msg__MotionPermission__Sequence__fini(rescue_interfaces__msg__MotionPermission__Sequence * array)
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
      rescue_interfaces__msg__MotionPermission__fini(&array->data[i]);
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

rescue_interfaces__msg__MotionPermission__Sequence *
rescue_interfaces__msg__MotionPermission__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__MotionPermission__Sequence * array = (rescue_interfaces__msg__MotionPermission__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__MotionPermission__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__MotionPermission__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__MotionPermission__Sequence__destroy(rescue_interfaces__msg__MotionPermission__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__MotionPermission__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__MotionPermission__Sequence__are_equal(const rescue_interfaces__msg__MotionPermission__Sequence * lhs, const rescue_interfaces__msg__MotionPermission__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__MotionPermission__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__MotionPermission__Sequence__copy(
  const rescue_interfaces__msg__MotionPermission__Sequence * input,
  rescue_interfaces__msg__MotionPermission__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__MotionPermission);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__MotionPermission * data =
      (rescue_interfaces__msg__MotionPermission *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__MotionPermission__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__MotionPermission__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__MotionPermission__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
