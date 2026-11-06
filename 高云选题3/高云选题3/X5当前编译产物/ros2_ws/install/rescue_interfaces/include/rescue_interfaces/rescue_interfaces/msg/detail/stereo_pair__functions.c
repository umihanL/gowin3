// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/StereoPair.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/stereo_pair__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `left`
// Member `right`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
rescue_interfaces__msg__StereoPair__init(rescue_interfaces__msg__StereoPair * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rescue_interfaces__msg__StereoPair__fini(msg);
    return false;
  }
  // sequence
  // left
  if (!sensor_msgs__msg__Image__init(&msg->left)) {
    rescue_interfaces__msg__StereoPair__fini(msg);
    return false;
  }
  // right
  if (!sensor_msgs__msg__Image__init(&msg->right)) {
    rescue_interfaces__msg__StereoPair__fini(msg);
    return false;
  }
  // synchronization_verified
  // skew_s
  return true;
}

void
rescue_interfaces__msg__StereoPair__fini(rescue_interfaces__msg__StereoPair * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sequence
  // left
  sensor_msgs__msg__Image__fini(&msg->left);
  // right
  sensor_msgs__msg__Image__fini(&msg->right);
  // synchronization_verified
  // skew_s
}

bool
rescue_interfaces__msg__StereoPair__are_equal(const rescue_interfaces__msg__StereoPair * lhs, const rescue_interfaces__msg__StereoPair * rhs)
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
  // sequence
  if (lhs->sequence != rhs->sequence) {
    return false;
  }
  // left
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->left), &(rhs->left)))
  {
    return false;
  }
  // right
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->right), &(rhs->right)))
  {
    return false;
  }
  // synchronization_verified
  if (lhs->synchronization_verified != rhs->synchronization_verified) {
    return false;
  }
  // skew_s
  if (lhs->skew_s != rhs->skew_s) {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__StereoPair__copy(
  const rescue_interfaces__msg__StereoPair * input,
  rescue_interfaces__msg__StereoPair * output)
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
  // sequence
  output->sequence = input->sequence;
  // left
  if (!sensor_msgs__msg__Image__copy(
      &(input->left), &(output->left)))
  {
    return false;
  }
  // right
  if (!sensor_msgs__msg__Image__copy(
      &(input->right), &(output->right)))
  {
    return false;
  }
  // synchronization_verified
  output->synchronization_verified = input->synchronization_verified;
  // skew_s
  output->skew_s = input->skew_s;
  return true;
}

rescue_interfaces__msg__StereoPair *
rescue_interfaces__msg__StereoPair__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__StereoPair * msg = (rescue_interfaces__msg__StereoPair *)allocator.allocate(sizeof(rescue_interfaces__msg__StereoPair), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__StereoPair));
  bool success = rescue_interfaces__msg__StereoPair__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__StereoPair__destroy(rescue_interfaces__msg__StereoPair * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__StereoPair__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__StereoPair__Sequence__init(rescue_interfaces__msg__StereoPair__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__StereoPair * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__StereoPair *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__StereoPair), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__StereoPair__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__StereoPair__fini(&data[i - 1]);
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
rescue_interfaces__msg__StereoPair__Sequence__fini(rescue_interfaces__msg__StereoPair__Sequence * array)
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
      rescue_interfaces__msg__StereoPair__fini(&array->data[i]);
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

rescue_interfaces__msg__StereoPair__Sequence *
rescue_interfaces__msg__StereoPair__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__StereoPair__Sequence * array = (rescue_interfaces__msg__StereoPair__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__StereoPair__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__StereoPair__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__StereoPair__Sequence__destroy(rescue_interfaces__msg__StereoPair__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__StereoPair__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__StereoPair__Sequence__are_equal(const rescue_interfaces__msg__StereoPair__Sequence * lhs, const rescue_interfaces__msg__StereoPair__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__StereoPair__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__StereoPair__Sequence__copy(
  const rescue_interfaces__msg__StereoPair__Sequence * input,
  rescue_interfaces__msg__StereoPair__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__StereoPair);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__StereoPair * data =
      (rescue_interfaces__msg__StereoPair *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__StereoPair__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__StereoPair__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__StereoPair__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
