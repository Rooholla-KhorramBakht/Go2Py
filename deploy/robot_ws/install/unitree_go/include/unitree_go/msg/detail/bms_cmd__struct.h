// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__BMS_CMD__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__BMS_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/BmsCmd in the package unitree_go.
typedef struct unitree_go__msg__BmsCmd
{
  uint8_t off;
  uint8_t reserve[3];
} unitree_go__msg__BmsCmd;

// Struct for a sequence of unitree_go__msg__BmsCmd.
typedef struct unitree_go__msg__BmsCmd__Sequence
{
  unitree_go__msg__BmsCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__BmsCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__BMS_CMD__STRUCT_H_
