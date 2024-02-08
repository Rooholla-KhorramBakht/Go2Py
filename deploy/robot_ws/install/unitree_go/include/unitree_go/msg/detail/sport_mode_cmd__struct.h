// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bms_cmd'
#include "unitree_go/msg/detail/bms_cmd__struct.h"
// Member 'path_point'
#include "unitree_go/msg/detail/path_point__struct.h"

// Struct defined in msg/SportModeCmd in the package unitree_go.
typedef struct unitree_go__msg__SportModeCmd
{
  uint8_t mode;
  uint8_t gait_type;
  uint8_t speed_level;
  float foot_raise_height;
  float body_height;
  float position[2];
  float euler[3];
  float velocity[2];
  float yaw_speed;
  unitree_go__msg__BmsCmd bms_cmd;
  unitree_go__msg__PathPoint path_point[30];
} unitree_go__msg__SportModeCmd;

// Struct for a sequence of unitree_go__msg__SportModeCmd.
typedef struct unitree_go__msg__SportModeCmd__Sequence
{
  unitree_go__msg__SportModeCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__SportModeCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__STRUCT_H_
