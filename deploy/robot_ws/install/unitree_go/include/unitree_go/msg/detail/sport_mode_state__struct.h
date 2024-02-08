// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/SportModeState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "unitree_go/msg/detail/time_spec__struct.h"
// Member 'imu_state'
#include "unitree_go/msg/detail/imu_state__struct.h"

// Struct defined in msg/SportModeState in the package unitree_go.
typedef struct unitree_go__msg__SportModeState
{
  unitree_go__msg__TimeSpec stamp;
  uint32_t error_code;
  unitree_go__msg__IMUState imu_state;
  uint8_t mode;
  float progress;
  uint8_t gait_type;
  float foot_raise_height;
  float position[3];
  float body_height;
  float velocity[3];
  float yaw_speed;
  float range_obstacle[4];
  int16_t foot_force[4];
  float foot_position_body[12];
  float foot_speed_body[12];
} unitree_go__msg__SportModeState;

// Struct for a sequence of unitree_go__msg__SportModeState.
typedef struct unitree_go__msg__SportModeState__Sequence
{
  unitree_go__msg__SportModeState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__SportModeState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__STRUCT_H_
