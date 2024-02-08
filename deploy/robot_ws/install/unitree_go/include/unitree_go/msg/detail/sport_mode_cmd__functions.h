// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__FUNCTIONS_H_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "unitree_go/msg/rosidl_generator_c__visibility_control.h"

#include "unitree_go/msg/detail/sport_mode_cmd__struct.h"

/// Initialize msg/SportModeCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * unitree_go__msg__SportModeCmd
 * )) before or use
 * unitree_go__msg__SportModeCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__SportModeCmd__init(unitree_go__msg__SportModeCmd * msg);

/// Finalize msg/SportModeCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__SportModeCmd__fini(unitree_go__msg__SportModeCmd * msg);

/// Create msg/SportModeCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * unitree_go__msg__SportModeCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
unitree_go__msg__SportModeCmd *
unitree_go__msg__SportModeCmd__create();

/// Destroy msg/SportModeCmd message.
/**
 * It calls
 * unitree_go__msg__SportModeCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__SportModeCmd__destroy(unitree_go__msg__SportModeCmd * msg);

/// Check for msg/SportModeCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__SportModeCmd__are_equal(const unitree_go__msg__SportModeCmd * lhs, const unitree_go__msg__SportModeCmd * rhs);

/// Copy a msg/SportModeCmd message.
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
unitree_go__msg__SportModeCmd__copy(
  const unitree_go__msg__SportModeCmd * input,
  unitree_go__msg__SportModeCmd * output);

/// Initialize array of msg/SportModeCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * unitree_go__msg__SportModeCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__SportModeCmd__Sequence__init(unitree_go__msg__SportModeCmd__Sequence * array, size_t size);

/// Finalize array of msg/SportModeCmd messages.
/**
 * It calls
 * unitree_go__msg__SportModeCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__SportModeCmd__Sequence__fini(unitree_go__msg__SportModeCmd__Sequence * array);

/// Create array of msg/SportModeCmd messages.
/**
 * It allocates the memory for the array and calls
 * unitree_go__msg__SportModeCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
unitree_go__msg__SportModeCmd__Sequence *
unitree_go__msg__SportModeCmd__Sequence__create(size_t size);

/// Destroy array of msg/SportModeCmd messages.
/**
 * It calls
 * unitree_go__msg__SportModeCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__SportModeCmd__Sequence__destroy(unitree_go__msg__SportModeCmd__Sequence * array);

/// Check for msg/SportModeCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__SportModeCmd__Sequence__are_equal(const unitree_go__msg__SportModeCmd__Sequence * lhs, const unitree_go__msg__SportModeCmd__Sequence * rhs);

/// Copy an array of msg/SportModeCmd messages.
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
unitree_go__msg__SportModeCmd__Sequence__copy(
  const unitree_go__msg__SportModeCmd__Sequence * input,
  unitree_go__msg__SportModeCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__FUNCTIONS_H_
