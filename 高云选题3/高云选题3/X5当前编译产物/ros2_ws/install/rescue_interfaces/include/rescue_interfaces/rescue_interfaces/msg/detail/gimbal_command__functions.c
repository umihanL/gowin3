// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/gimbal_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `owner`
#include "rosidl_runtime_c/string_functions.h"

bool
rescue_interfaces__msg__GimbalCommand__init(rescue_interfaces__msg__GimbalCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rescue_interfaces__msg__GimbalCommand__fini(msg);
    return false;
  }
  // owner
  if (!rosidl_runtime_c__String__init(&msg->owner)) {
    rescue_interfaces__msg__GimbalCommand__fini(msg);
    return false;
  }
  // pan_deg
  // tilt_deg
  // speed_deg_s
  // lease_ms
  return true;
}

void
rescue_interfaces__msg__GimbalCommand__fini(rescue_interfaces__msg__GimbalCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // owner
  rosidl_runtime_c__String__fini(&msg->owner);
  // pan_deg
  // tilt_deg
  // speed_deg_s
  // lease_ms
}

bool
rescue_interfaces__msg__GimbalCommand__are_equal(const rescue_interfaces__msg__GimbalCommand * lhs, const rescue_interfaces__msg__GimbalCommand * rhs)
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
  // owner
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->owner), &(rhs->owner)))
  {
    return false;
  }
  // pan_deg
  if (lhs->pan_deg != rhs->pan_deg) {
    return false;
  }
  // tilt_deg
  if (lhs->tilt_deg != rhs->tilt_deg) {
    return false;
  }
  // speed_deg_s
  if (lhs->speed_deg_s != rhs->speed_deg_s) {
    return false;
  }
  // lease_ms
  if (lhs->lease_ms != rhs->lease_ms) {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__GimbalCommand__copy(
  const rescue_interfaces__msg__GimbalCommand * input,
  rescue_interfaces__msg__GimbalCommand * output)
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
  // owner
  if (!rosidl_runtime_c__String__copy(
      &(input->owner), &(output->owner)))
  {
    return false;
  }
  // pan_deg
  output->pan_deg = input->pan_deg;
  // tilt_deg
  output->tilt_deg = input->tilt_deg;
  // speed_deg_s
  output->speed_deg_s = input->speed_deg_s;
  // lease_ms
  output->lease_ms = input->lease_ms;
  return true;
}

rescue_interfaces__msg__GimbalCommand *
rescue_interfaces__msg__GimbalCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__GimbalCommand * msg = (rescue_interfaces__msg__GimbalCommand *)allocator.allocate(sizeof(rescue_interfaces__msg__GimbalCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__GimbalCommand));
  bool success = rescue_interfaces__msg__GimbalCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__GimbalCommand__destroy(rescue_interfaces__msg__GimbalCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__GimbalCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__GimbalCommand__Sequence__init(rescue_interfaces__msg__GimbalCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__GimbalCommand * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__GimbalCommand *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__GimbalCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__GimbalCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__GimbalCommand__fini(&data[i - 1]);
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
rescue_interfaces__msg__GimbalCommand__Sequence__fini(rescue_interfaces__msg__GimbalCommand__Sequence * array)
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
      rescue_interfaces__msg__GimbalCommand__fini(&array->data[i]);
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

rescue_interfaces__msg__GimbalCommand__Sequence *
rescue_interfaces__msg__GimbalCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__GimbalCommand__Sequence * array = (rescue_interfaces__msg__GimbalCommand__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__GimbalCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__GimbalCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__GimbalCommand__Sequence__destroy(rescue_interfaces__msg__GimbalCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__GimbalCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__GimbalCommand__Sequence__are_equal(const rescue_interfaces__msg__GimbalCommand__Sequence * lhs, const rescue_interfaces__msg__GimbalCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__GimbalCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__GimbalCommand__Sequence__copy(
  const rescue_interfaces__msg__GimbalCommand__Sequence * input,
  rescue_interfaces__msg__GimbalCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__GimbalCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__GimbalCommand * data =
      (rescue_interfaces__msg__GimbalCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__GimbalCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__GimbalCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__GimbalCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
