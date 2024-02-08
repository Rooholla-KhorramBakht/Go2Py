// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_HEADER__STRUCT_H_
#define UNITREE_API__MSG__DETAIL__REQUEST_HEADER__STRUCT_H_

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
// Member 'lease'
#include "unitree_api/msg/detail/request_lease__struct.h"
// Member 'policy'
#include "unitree_api/msg/detail/request_policy__struct.h"

// Struct defined in msg/RequestHeader in the package unitree_api.
typedef struct unitree_api__msg__RequestHeader
{
  unitree_api__msg__RequestIdentity identity;
  unitree_api__msg__RequestLease lease;
  unitree_api__msg__RequestPolicy policy;
} unitree_api__msg__RequestHeader;

// Struct for a sequence of unitree_api__msg__RequestHeader.
typedef struct unitree_api__msg__RequestHeader__Sequence
{
  unitree_api__msg__RequestHeader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_api__msg__RequestHeader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_HEADER__STRUCT_H_
