// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_api:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE__STRUCT_H_
#define UNITREE_API__MSG__DETAIL__RESPONSE__STRUCT_H_

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
#include "unitree_api/msg/detail/response_header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/string.h"
// Member 'binary'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Response in the package unitree_api.
typedef struct unitree_api__msg__Response
{
  unitree_api__msg__ResponseHeader header;
  rosidl_runtime_c__String data;
  rosidl_runtime_c__int8__Sequence binary;
} unitree_api__msg__Response;

// Struct for a sequence of unitree_api__msg__Response.
typedef struct unitree_api__msg__Response__Sequence
{
  unitree_api__msg__Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_api__msg__Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE__STRUCT_H_
