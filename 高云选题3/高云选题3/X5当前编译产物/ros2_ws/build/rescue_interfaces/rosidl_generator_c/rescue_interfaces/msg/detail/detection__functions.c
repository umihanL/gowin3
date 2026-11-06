// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `class_name`
// Member `depth_reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `position_m`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
rescue_interfaces__msg__Detection__init(rescue_interfaces__msg__Detection * msg)
{
  if (!msg) {
    return false;
  }
  // class_id
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    rescue_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // confidence
  // bbox_xyxy
  // depth_valid
  // position_m
  if (!geometry_msgs__msg__Point__init(&msg->position_m)) {
    rescue_interfaces__msg__Detection__fini(msg);
    return false;
  }
  // depth_m
  // depth_reason
  if (!rosidl_runtime_c__String__init(&msg->depth_reason)) {
    rescue_interfaces__msg__Detection__fini(msg);
    return false;
  }
  return true;
}

void
rescue_interfaces__msg__Detection__fini(rescue_interfaces__msg__Detection * msg)
{
  if (!msg) {
    return;
  }
  // class_id
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // confidence
  // bbox_xyxy
  // depth_valid
  // position_m
  geometry_msgs__msg__Point__fini(&msg->position_m);
  // depth_m
  // depth_reason
  rosidl_runtime_c__String__fini(&msg->depth_reason);
}

bool
rescue_interfaces__msg__Detection__are_equal(const rescue_interfaces__msg__Detection * lhs, const rescue_interfaces__msg__Detection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // bbox_xyxy
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->bbox_xyxy[i] != rhs->bbox_xyxy[i]) {
      return false;
    }
  }
  // depth_valid
  if (lhs->depth_valid != rhs->depth_valid) {
    return false;
  }
  // position_m
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position_m), &(rhs->position_m)))
  {
    return false;
  }
  // depth_m
  if (lhs->depth_m != rhs->depth_m) {
    return false;
  }
  // depth_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->depth_reason), &(rhs->depth_reason)))
  {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__Detection__copy(
  const rescue_interfaces__msg__Detection * input,
  rescue_interfaces__msg__Detection * output)
{
  if (!input || !output) {
    return false;
  }
  // class_id
  output->class_id = input->class_id;
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // bbox_xyxy
  for (size_t i = 0; i < 4; ++i) {
    output->bbox_xyxy[i] = input->bbox_xyxy[i];
  }
  // depth_valid
  output->depth_valid = input->depth_valid;
  // position_m
  if (!geometry_msgs__msg__Point__copy(
      &(input->position_m), &(output->position_m)))
  {
    return false;
  }
  // depth_m
  output->depth_m = input->depth_m;
  // depth_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->depth_reason), &(output->depth_reason)))
  {
    return false;
  }
  return true;
}

rescue_interfaces__msg__Detection *
rescue_interfaces__msg__Detection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__Detection * msg = (rescue_interfaces__msg__Detection *)allocator.allocate(sizeof(rescue_interfaces__msg__Detection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__Detection));
  bool success = rescue_interfaces__msg__Detection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__Detection__destroy(rescue_interfaces__msg__Detection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__Detection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__Detection__Sequence__init(rescue_interfaces__msg__Detection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__Detection * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__Detection *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__Detection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__Detection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__Detection__fini(&data[i - 1]);
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
rescue_interfaces__msg__Detection__Sequence__fini(rescue_interfaces__msg__Detection__Sequence * array)
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
      rescue_interfaces__msg__Detection__fini(&array->data[i]);
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

rescue_interfaces__msg__Detection__Sequence *
rescue_interfaces__msg__Detection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__Detection__Sequence * array = (rescue_interfaces__msg__Detection__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__Detection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__Detection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__Detection__Sequence__destroy(rescue_interfaces__msg__Detection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__Detection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__Detection__Sequence__are_equal(const rescue_interfaces__msg__Detection__Sequence * lhs, const rescue_interfaces__msg__Detection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__Detection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__Detection__Sequence__copy(
  const rescue_interfaces__msg__Detection__Sequence * input,
  rescue_interfaces__msg__Detection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__Detection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__Detection * data =
      (rescue_interfaces__msg__Detection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__Detection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__Detection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__Detection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
