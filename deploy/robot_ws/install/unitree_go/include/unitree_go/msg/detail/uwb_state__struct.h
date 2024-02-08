// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/UwbState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__UWB_STATE__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__UWB_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/UwbState in the package unitree_go.
typedef struct unitree_go__msg__UwbState
{
  uint8_t version[2];
  uint8_t channel;
  uint8_t joy_mode;
  float orientation_est;
  float pitch_est;
  float distance_est;
  float yaw_est;
  float tag_roll;
  float tag_pitch;
  float tag_yaw;
  float base_roll;
  float base_pitch;
  float base_yaw;
  float joystick[2];
  uint8_t error_state;
  uint8_t buttons;
  uint8_t enabled_from_app;
} unitree_go__msg__UwbState;

// Struct for a sequence of unitree_go__msg__UwbState.
typedef struct unitree_go__msg__UwbState__Sequence
{
  unitree_go__msg__UwbState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__UwbState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__UWB_STATE__STRUCT_H_
