// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/FpgaEvent.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/fpga_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rescue_interfaces__msg__FpgaEvent__init(rescue_interfaces__msg__FpgaEvent * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rescue_interfaces__msg__FpgaEvent__fini(msg);
    return false;
  }
  // event_type
  // wire_sequence
  // event_id
  // vehicle_stopped
  // gimbal_settled
  // measured_angles_valid
  // pan_deg
  // tilt_deg
  // acknowledged_sequence
  // acknowledgement_result
  return true;
}

void
rescue_interfaces__msg__FpgaEvent__fini(rescue_interfaces__msg__FpgaEvent * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // event_type
  // wire_sequence
  // event_id
  // vehicle_stopped
  // gimbal_settled
  // measured_angles_valid
  // pan_deg
  // tilt_deg
  // acknowledged_sequence
  // acknowledgement_result
}

bool
rescue_interfaces__msg__FpgaEvent__are_equal(const rescue_interfaces__msg__FpgaEvent * lhs, const rescue_interfaces__msg__FpgaEvent * rhs)
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
  // event_type
  if (lhs->event_type != rhs->event_type) {
    return false;
  }
  // wire_sequence
  if (lhs->wire_sequence != rhs->wire_sequence) {
    return false;
  }
  // event_id
  if (lhs->event_id != rhs->event_id) {
    return false;
  }
  // vehicle_stopped
  if (lhs->vehicle_stopped != rhs->vehicle_stopped) {
    return false;
  }
  // gimbal_settled
  if (lhs->gimbal_settled != rhs->gimbal_settled) {
    return false;
  }
  // measured_angles_valid
  if (lhs->measured_angles_valid != rhs->measured_angles_valid) {
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
  // acknowledged_sequence
  if (lhs->acknowledged_sequence != rhs->acknowledged_sequence) {
    return false;
  }
  // acknowledgement_result
  if (lhs->acknowledgement_result != rhs->acknowledgement_result) {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__FpgaEvent__copy(
  const rescue_interfaces__msg__FpgaEvent * input,
  rescue_interfaces__msg__FpgaEvent * output)
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
  // event_type
  output->event_type = input->event_type;
  // wire_sequence
  output->wire_sequence = input->wire_sequence;
  // event_id
  output->event_id = input->event_id;
  // vehicle_stopped
  output->vehicle_stopped = input->vehicle_stopped;
  // gimbal_settled
  output->gimbal_settled = input->gimbal_settled;
  // measured_angles_valid
  output->measured_angles_valid = input->measured_angles_valid;
  // pan_deg
  output->pan_deg = input->pan_deg;
  // tilt_deg
  output->tilt_deg = input->tilt_deg;
  // acknowledged_sequence
  output->acknowledged_sequence = input->acknowledged_sequence;
  // acknowledgement_result
  output->acknowledgement_result = input->acknowledgement_result;
  return true;
}

rescue_interfaces__msg__FpgaEvent *
rescue_interfaces__msg__FpgaEvent__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__FpgaEvent * msg = (rescue_interfaces__msg__FpgaEvent *)allocator.allocate(sizeof(rescue_interfaces__msg__FpgaEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__FpgaEvent));
  bool success = rescue_interfaces__msg__FpgaEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__FpgaEvent__destroy(rescue_interfaces__msg__FpgaEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__FpgaEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__FpgaEvent__Sequence__init(rescue_interfaces__msg__FpgaEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__FpgaEvent * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__FpgaEvent *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__FpgaEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__FpgaEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__FpgaEvent__fini(&data[i - 1]);
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
rescue_interfaces__msg__FpgaEvent__Sequence__fini(rescue_interfaces__msg__FpgaEvent__Sequence * array)
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
      rescue_interfaces__msg__FpgaEvent__fini(&array->data[i]);
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

rescue_interfaces__msg__FpgaEvent__Sequence *
rescue_interfaces__msg__FpgaEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__FpgaEvent__Sequence * array = (rescue_interfaces__msg__FpgaEvent__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__FpgaEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__FpgaEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__FpgaEvent__Sequence__destroy(rescue_interfaces__msg__FpgaEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__FpgaEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__FpgaEvent__Sequence__are_equal(const rescue_interfaces__msg__FpgaEvent__Sequence * lhs, const rescue_interfaces__msg__FpgaEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__FpgaEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__FpgaEvent__Sequence__copy(
  const rescue_interfaces__msg__FpgaEvent__Sequence * input,
  rescue_interfaces__msg__FpgaEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__FpgaEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__FpgaEvent * data =
      (rescue_interfaces__msg__FpgaEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__FpgaEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__FpgaEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__FpgaEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
