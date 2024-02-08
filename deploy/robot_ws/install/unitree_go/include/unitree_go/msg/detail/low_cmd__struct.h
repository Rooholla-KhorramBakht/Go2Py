// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_CMD__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__LOW_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_cmd'
#include "unitree_go/msg/detail/motor_cmd__struct.h"
// Member 'bms_cmd'
#include "unitree_go/msg/detail/bms_cmd__struct.h"

// Struct defined in msg/LowCmd in the package unitree_go.
typedef struct unitree_go__msg__LowCmd
{
  uint8_t head[2];
  uint8_t level_flag;
  uint8_t frame_reserve;
  uint32_t sn[2];
  uint32_t version[2];
  uint16_t bandwidth;
  unitree_go__msg__MotorCmd motor_cmd[20];
  unitree_go__msg__BmsCmd bms_cmd;
  uint8_t wireless_remote[40];
  uint8_t led[12];
  uint8_t fan[2];
  uint8_t gpio;
  uint32_t reserve;
  uint32_t crc;
} unitree_go__msg__LowCmd;

// Struct for a sequence of unitree_go__msg__LowCmd.
typedef struct unitree_go__msg__LowCmd__Sequence
{
  unitree_go__msg__LowCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__LowCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__LOW_CMD__STRUCT_H_
