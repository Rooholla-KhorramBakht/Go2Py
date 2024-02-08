// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from unitree_go:msg/HeightMap.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__FUNCTIONS_H_
#define UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "unitree_go/msg/rosidl_generator_c__visibility_control.h"

#include "unitree_go/msg/detail/height_map__struct.h"

/// Initialize msg/HeightMap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * unitree_go__msg__HeightMap
 * )) before or use
 * unitree_go__msg__HeightMap__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__HeightMap__init(unitree_go__msg__HeightMap * msg);

/// Finalize msg/HeightMap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__HeightMap__fini(unitree_go__msg__HeightMap * msg);

/// Create msg/HeightMap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * unitree_go__msg__HeightMap__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
unitree_go__msg__HeightMap *
unitree_go__msg__HeightMap__create();

/// Destroy msg/HeightMap message.
/**
 * It calls
 * unitree_go__msg__HeightMap__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__HeightMap__destroy(unitree_go__msg__HeightMap * msg);

/// Check for msg/HeightMap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__HeightMap__are_equal(const unitree_go__msg__HeightMap * lhs, const unitree_go__msg__HeightMap * rhs);

/// Copy a msg/HeightMap message.
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
unitree_go__msg__HeightMap__copy(
  const unitree_go__msg__HeightMap * input,
  unitree_go__msg__HeightMap * output);

/// Initialize array of msg/HeightMap messages.
/**
 * It allocates the memory for the number of elements and calls
 * unitree_go__msg__HeightMap__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__HeightMap__Sequence__init(unitree_go__msg__HeightMap__Sequence * array, size_t size);

/// Finalize array of msg/HeightMap messages.
/**
 * It calls
 * unitree_go__msg__HeightMap__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__HeightMap__Sequence__fini(unitree_go__msg__HeightMap__Sequence * array);

/// Create array of msg/HeightMap messages.
/**
 * It allocates the memory for the array and calls
 * unitree_go__msg__HeightMap__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
unitree_go__msg__HeightMap__Sequence *
unitree_go__msg__HeightMap__Sequence__create(size_t size);

/// Destroy array of msg/HeightMap messages.
/**
 * It calls
 * unitree_go__msg__HeightMap__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
void
unitree_go__msg__HeightMap__Sequence__destroy(unitree_go__msg__HeightMap__Sequence * array);

/// Check for msg/HeightMap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_unitree_go
bool
unitree_go__msg__HeightMap__Sequence__are_equal(const unitree_go__msg__HeightMap__Sequence * lhs, const unitree_go__msg__HeightMap__Sequence * rhs);

/// Copy an array of msg/HeightMap messages.
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
unitree_go__msg__HeightMap__Sequence__copy(
  const unitree_go__msg__HeightMap__Sequence * input,
  unitree_go__msg__HeightMap__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__FUNCTIONS_H_
