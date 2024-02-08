// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_api:msg/Request.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST__STRUCT_H_
#define UNITREE_API__MSG__DETAIL__REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "unitree_api/msg/detail/request_header__struct.h"
// Member 'parameter'
#include "rosidl_runtime_c/string.h"
// Member 'binary'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Request in the package unitree_api.
typedef struct unitree_api__msg__Request
{
  unitree_api__msg__RequestHeader header;
  rosidl_runtime_c__String parameter;
  rosidl_runtime_c__uint8__Sequence binary;
} unitree_api__msg__Request;

// Struct for a sequence of unitree_api__msg__Request.
typedef struct unitree_api__msg__Request__Sequence
{
  unitree_api__msg__Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_api__msg__Request__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_API__MSG__DETAIL__REQUEST__STRUCT_H_
