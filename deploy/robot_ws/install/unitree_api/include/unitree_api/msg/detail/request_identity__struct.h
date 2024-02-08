// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_api:msg/RequestIdentity.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__STRUCT_H_
#define UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RequestIdentity in the package unitree_api.
typedef struct unitree_api__msg__RequestIdentity
{
  int64_t id;
  int64_t api_id;
} unitree_api__msg__RequestIdentity;

// Struct for a sequence of unitree_api__msg__RequestIdentity.
typedef struct unitree_api__msg__RequestIdentity__Sequence
{
  unitree_api__msg__RequestIdentity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_api__msg__RequestIdentity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__STRUCT_H_
