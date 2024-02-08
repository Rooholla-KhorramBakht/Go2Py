// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from unitree_go:msg/Res.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__RES__FUNCTIONS_H_
#define UNITREE_GO__MSG__DETAIL__RES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "unitree_go/msg/rosidl_generator_c__visibility_control.h"

#include "unitree_go/msg/detail/res__struct.h"

/// Initialize msg/Res message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * unitree_go__msg__Res
 * )) before or use
 * unitree_go__msg__Res__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__Res__init(unitree_go__msg__Res * msg);

/// Finalize msg/Res message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__Res__fini(unitree_go__msg__Res * msg);

/// Create msg/Res message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * unitree_go__msg__Res__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
unitree_go__msg__Res *
unitree_go__msg__Res__create();

/// Destroy msg/Res message.
/**
 * It calls
 * unitree_go__msg__Res__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__Res__destroy(unitree_go__msg__Res * msg);

/// Check for msg/Res message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__Res__are_equal(const unitree_go__msg__Res * lhs, const unitree_go__msg__Res * rhs);

/// Copy a msg/Res message.
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
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__Res__copy(
  const unitree_go__msg__Res * input,
  unitree_go__msg__Res * output);

/// Initialize array of msg/Res messages.
/**
 * It allocates the memory for the number of elements and calls
 * unitree_go__msg__Res__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__Res__Sequence__init(unitree_go__msg__Res__Sequence * array, size_t size);

/// Finalize array of msg/Res messages.
/**
 * It calls
 * unitree_go__msg__Res__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__Res__Sequence__fini(unitree_go__msg__Res__Sequence * array);

/// Create array of msg/Res messages.
/**
 * It allocates the memory for the array and calls
 * unitree_go__msg__Res__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
unitree_go__msg__Res__Sequence *
unitree_go__msg__Res__Sequence__create(size_t size);

/// Destroy array of msg/Res messages.
/**
 * It calls
 * unitree_go__msg__Res__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__Res__Sequence__destroy(unitree_go__msg__Res__Sequence * array);

/// Check for msg/Res message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__Res__Sequence__are_equal(const unitree_go__msg__Res__Sequence * lhs, const unitree_go__msg__Res__Sequence * rhs);

/// Copy an array of msg/Res messages.
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
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__Res__Sequence__copy(
  const unitree_go__msg__Res__Sequence * input,
  unitree_go__msg__Res__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__RES__FUNCTIONS_H_
