// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'video720p'
// Member 'video360p'
// Member 'video180p'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Go2FrontVideoData in the package unitree_go.
typedef struct unitree_go__msg__Go2FrontVideoData
{
  uint64_t time_frame;
  rosidl_runtime_c__uint8__Sequence video720p;
  rosidl_runtime_c__uint8__Sequence video360p;
  rosidl_runtime_c__uint8__Sequence video180p;
} unitree_go__msg__Go2FrontVideoData;

// Struct for a sequence of unitree_go__msg__Go2FrontVideoData.
typedef struct unitree_go__msg__Go2FrontVideoData__Sequence
{
  unitree_go__msg__Go2FrontVideoData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__Go2FrontVideoData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_H_
