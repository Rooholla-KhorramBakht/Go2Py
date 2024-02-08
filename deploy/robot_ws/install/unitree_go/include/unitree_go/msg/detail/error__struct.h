// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__ERROR__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Error in the package unitree_go.
typedef struct unitree_go__msg__Error
{
  uint32_t source;
  uint32_t state;
} unitree_go__msg__Error;

// Struct for a sequence of unitree_go__msg__Error.
typedef struct unitree_go__msg__Error__Sequence
{
  unitree_go__msg__Error * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__Error__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__ERROR__STRUCT_H_
