// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/Req.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__REQ__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__REQ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'uuid'
// Member 'body'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Req in the package unitree_go.
typedef struct unitree_go__msg__Req
{
  rosidl_runtime_c__String uuid;
  rosidl_runtime_c__String body;
} unitree_go__msg__Req;

// Struct for a sequence of unitree_go__msg__Req.
typedef struct unitree_go__msg__Req__Sequence
{
  unitree_go__msg__Req * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__Req__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__REQ__STRUCT_H_
