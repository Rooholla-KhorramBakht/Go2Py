// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/AudioData.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__AUDIO_DATA__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__AUDIO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/AudioData in the package unitree_go.
typedef struct unitree_go__msg__AudioData
{
  uint64_t time_frame;
  rosidl_runtime_c__uint8__Sequence data;
} unitree_go__msg__AudioData;

// Struct for a sequence of unitree_go__msg__AudioData.
typedef struct unitree_go__msg__AudioData__Sequence
{
  unitree_go__msg__AudioData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__AudioData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__AUDIO_DATA__STRUCT_H_
