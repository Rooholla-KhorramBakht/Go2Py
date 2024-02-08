// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_api:msg/ResponseHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__STRUCT_H_
#define UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'identity'
#include "unitree_api/msg/detail/request_identity__struct.h"
// Member 'status'
#include "unitree_api/msg/detail/response_status__struct.h"

// Struct defined in msg/ResponseHeader in the package unitree_api.
typedef struct unitree_api__msg__ResponseHeader
{
  unitree_api__msg__RequestIdentity identity;
  unitree_api__msg__ResponseStatus status;
} unitree_api__msg__ResponseHeader;

// Struct for a sequence of unitree_api__msg__ResponseHeader.
typedef struct unitree_api__msg__ResponseHeader__Sequence
{
  unitree_api__msg__ResponseHeader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_api__msg__ResponseHeader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__STRUCT_H_
