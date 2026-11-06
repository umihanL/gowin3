// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rescue_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__FUNCTIONS_H_
#define RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rescue_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "rescue_interfaces/msg/detail/gimbal_command__struct.h"

/// Initialize msg/GimbalCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rescue_interfaces__msg__GimbalCommand
 * )) before or use
 * rescue_interfaces__msg__GimbalCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
bool
rescue_interfaces__msg__GimbalCommand__init(rescue_interfaces__msg__GimbalCommand * msg);

/// Finalize msg/GimbalCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
void
rescue_interfaces__msg__GimbalCommand__fini(rescue_interfaces__msg__GimbalCommand * msg);

/// Create msg/GimbalCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rescue_interfaces__msg__GimbalCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
rescue_interfaces__msg__GimbalCommand *
rescue_interfaces__msg__GimbalCommand__create();

/// Destroy msg/GimbalCommand message.
/**
 * It calls
 * rescue_interfaces__msg__GimbalCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
void
rescue_interfaces__msg__GimbalCommand__destroy(rescue_interfaces__msg__GimbalCommand * msg);

/// Check for msg/GimbalCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
bool
rescue_interfaces__msg__GimbalCommand__are_equal(const rescue_interfaces__msg__GimbalCommand * lhs, const rescue_interfaces__msg__GimbalCommand * rhs);

/// Copy a msg/GimbalCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
bool
rescue_interfaces__msg__GimbalCommand__copy(
  const rescue_interfaces__msg__GimbalCommand * input,
  rescue_interfaces__msg__GimbalCommand * output);

/// Initialize array of msg/GimbalCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * rescue_interfaces__msg__GimbalCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
bool
rescue_interfaces__msg__GimbalCommand__Sequence__init(rescue_interfaces__msg__GimbalCommand__Sequence * array, size_t size);

/// Finalize array of msg/GimbalCommand messages.
/**
 * It calls
 * rescue_interfaces__msg__GimbalCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
void
rescue_interfaces__msg__GimbalCommand__Sequence__fini(rescue_interfaces__msg__GimbalCommand__Sequence * array);

/// Create array of msg/GimbalCommand messages.
/**
 * It allocates the memory for the array and calls
 * rescue_interfaces__msg__GimbalCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
rescue_interfaces__msg__GimbalCommand__Sequence *
rescue_interfaces__msg__GimbalCommand__Sequence__create(size_t size);

/// Destroy array of msg/GimbalCommand messages.
/**
 * It calls
 * rescue_interfaces__msg__GimbalCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
void
rescue_interfaces__msg__GimbalCommand__Sequence__destroy(rescue_interfaces__msg__GimbalCommand__Sequence * array);

/// Check for msg/GimbalCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
bool
rescue_interfaces__msg__GimbalCommand__Sequence__are_equal(const rescue_interfaces__msg__GimbalCommand__Sequence * lhs, const rescue_interfaces__msg__GimbalCommand__Sequence * rhs);

/// Copy an array of msg/GimbalCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rescue_interfaces
bool
rescue_interfaces__msg__GimbalCommand__Sequence__copy(
  const rescue_interfaces__msg__GimbalCommand__Sequence * input,
  rescue_interfaces__msg__GimbalCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RESCUE_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__FUNCTIONS_H_
