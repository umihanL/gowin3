// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rescue_interfaces:msg/DetectionArray.idl
// generated code does not contain a copyright notice
#include "rescue_interfaces/msg/detail/detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `status`
// Member `stereo_calibration_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `detections`
#include "rescue_interfaces/msg/detail/detection__functions.h"

bool
rescue_interfaces__msg__DetectionArray__init(rescue_interfaces__msg__DetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rescue_interfaces__msg__DetectionArray__fini(msg);
    return false;
  }
  // sequence
  // width
  // height
  // rectified
  // detector_available
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    rescue_interfaces__msg__DetectionArray__fini(msg);
    return false;
  }
  // stereo_calibration_id
  if (!rosidl_runtime_c__String__init(&msg->stereo_calibration_id)) {
    rescue_interfaces__msg__DetectionArray__fini(msg);
    return false;
  }
  // detections
  if (!rescue_interfaces__msg__Detection__Sequence__init(&msg->detections, 0)) {
    rescue_interfaces__msg__DetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
rescue_interfaces__msg__DetectionArray__fini(rescue_interfaces__msg__DetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sequence
  // width
  // height
  // rectified
  // detector_available
  // status
  rosidl_runtime_c__String__fini(&msg->status);
  // stereo_calibration_id
  rosidl_runtime_c__String__fini(&msg->stereo_calibration_id);
  // detections
  rescue_interfaces__msg__Detection__Sequence__fini(&msg->detections);
}

bool
rescue_interfaces__msg__DetectionArray__are_equal(const rescue_interfaces__msg__DetectionArray * lhs, const rescue_interfaces__msg__DetectionArray * rhs)
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
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // rectified
  if (lhs->rectified != rhs->rectified) {
    return false;
  }
  // detector_available
  if (lhs->detector_available != rhs->detector_available) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // stereo_calibration_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->stereo_calibration_id), &(rhs->stereo_calibration_id)))
  {
    return false;
  }
  // detections
  if (!rescue_interfaces__msg__Detection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
rescue_interfaces__msg__DetectionArray__copy(
  const rescue_interfaces__msg__DetectionArray * input,
  rescue_interfaces__msg__DetectionArray * output)
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
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // rectified
  output->rectified = input->rectified;
  // detector_available
  output->detector_available = input->detector_available;
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // stereo_calibration_id
  if (!rosidl_runtime_c__String__copy(
      &(input->stereo_calibration_id), &(output->stereo_calibration_id)))
  {
    return false;
  }
  // detections
  if (!rescue_interfaces__msg__Detection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

rescue_interfaces__msg__DetectionArray *
rescue_interfaces__msg__DetectionArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__DetectionArray * msg = (rescue_interfaces__msg__DetectionArray *)allocator.allocate(sizeof(rescue_interfaces__msg__DetectionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rescue_interfaces__msg__DetectionArray));
  bool success = rescue_interfaces__msg__DetectionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rescue_interfaces__msg__DetectionArray__destroy(rescue_interfaces__msg__DetectionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rescue_interfaces__msg__DetectionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rescue_interfaces__msg__DetectionArray__Sequence__init(rescue_interfaces__msg__DetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__DetectionArray * data = NULL;

  if (size) {
    data = (rescue_interfaces__msg__DetectionArray *)allocator.zero_allocate(size, sizeof(rescue_interfaces__msg__DetectionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rescue_interfaces__msg__DetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rescue_interfaces__msg__DetectionArray__fini(&data[i - 1]);
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
rescue_interfaces__msg__DetectionArray__Sequence__fini(rescue_interfaces__msg__DetectionArray__Sequence * array)
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
      rescue_interfaces__msg__DetectionArray__fini(&array->data[i]);
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

rescue_interfaces__msg__DetectionArray__Sequence *
rescue_interfaces__msg__DetectionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rescue_interfaces__msg__DetectionArray__Sequence * array = (rescue_interfaces__msg__DetectionArray__Sequence *)allocator.allocate(sizeof(rescue_interfaces__msg__DetectionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rescue_interfaces__msg__DetectionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rescue_interfaces__msg__DetectionArray__Sequence__destroy(rescue_interfaces__msg__DetectionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rescue_interfaces__msg__DetectionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rescue_interfaces__msg__DetectionArray__Sequence__are_equal(const rescue_interfaces__msg__DetectionArray__Sequence * lhs, const rescue_interfaces__msg__DetectionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rescue_interfaces__msg__DetectionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rescue_interfaces__msg__DetectionArray__Sequence__copy(
  const rescue_interfaces__msg__DetectionArray__Sequence * input,
  rescue_interfaces__msg__DetectionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rescue_interfaces__msg__DetectionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rescue_interfaces__msg__DetectionArray * data =
      (rescue_interfaces__msg__DetectionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rescue_interfaces__msg__DetectionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rescue_interfaces__msg__DetectionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rescue_interfaces__msg__DetectionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
