// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_STATE__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__LOW_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'imu_state'
#include "unitree_go/msg/detail/imu_state__struct.h"
// Member 'motor_state'
#include "unitree_go/msg/detail/motor_state__struct.h"
// Member 'bms_state'
#include "unitree_go/msg/detail/bms_state__struct.h"

// Struct defined in msg/LowState in the package unitree_go.
typedef struct unitree_go__msg__LowState
{
  uint8_t head[2];
  uint8_t level_flag;
  uint8_t frame_reserve;
  uint32_t sn[2];
  uint32_t version[2];
  uint16_t bandwidth;
  unitree_go__msg__IMUState imu_state;
  unitree_go__msg__MotorState motor_state[20];
  unitree_go__msg__BmsState bms_state;
  int16_t foot_force[4];
  int16_t foot_force_est[4];
  uint32_t tick;
  uint8_t wireless_remote[40];
  uint8_t bit_flag;
  float adc_reel;
  int8_t temperature_ntc1;
  int8_t temperature_ntc2;
  float power_v;
  float power_a;
  uint16_t fan_frequency[4];
  uint32_t reserve;
  uint32_t crc;
} unitree_go__msg__LowState;

// Struct for a sequence of unitree_go__msg__LowState.
typedef struct unitree_go__msg__LowState__Sequence
{
  unitree_go__msg__LowState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__LowState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__LOW_STATE__STRUCT_H_
