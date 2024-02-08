// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_api:msg/RequestLease.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_LEASE__STRUCT_H_
#define UNITREE_API__MSG__DETAIL__REQUEST_LEASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RequestLease in the package unitree_api.
typedef struct unitree_api__msg__RequestLease
{
  int64_t id;
} unitree_api__msg__RequestLease;

// Struct for a sequence of unitree_api__msg__RequestLease.
typedef struct unitree_api__msg__RequestLease__Sequence
{
  unitree_api__msg__RequestLease * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_api__msg__RequestLease__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_LEASE__STRUCT_H_
