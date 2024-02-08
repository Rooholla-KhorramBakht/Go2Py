// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/HeightMap.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame_id'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/HeightMap in the package unitree_go.
typedef struct unitree_go__msg__HeightMap
{
  double stamp;
  rosidl_runtime_c__String frame_id;
  float resolution;
  uint32_t width;
  uint32_t height;
  float origin[2];
  rosidl_runtime_c__float__Sequence data;
} unitree_go__msg__HeightMap;

// Struct for a sequence of unitree_go__msg__HeightMap.
typedef struct unitree_go__msg__HeightMap__Sequence
{
  unitree_go__msg__HeightMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__HeightMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__STRUCT_H_
