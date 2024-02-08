// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/LidarState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LIDAR_STATE__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__LIDAR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'firmware_version'
// Member 'software_version'
// Member 'sdk_version'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/LidarState in the package unitree_go.
typedef struct unitree_go__msg__LidarState
{
  double stamp;
  rosidl_runtime_c__String firmware_version;
  rosidl_runtime_c__String software_version;
  rosidl_runtime_c__String sdk_version;
  float sys_rotation_speed;
  float com_rotation_speed;
  uint8_t error_state;
  float cloud_frequency;
  float cloud_packet_loss_rate;
  uint32_t cloud_size;
  uint32_t cloud_scan_num;
  float imu_frequency;
  float imu_packet_loss_rate;
  float imu_rpy[3];
  double serial_recv_stamp;
  uint32_t serial_buffer_size;
  uint32_t serial_buffer_read;
} unitree_go__msg__LidarState;

// Struct for a sequence of unitree_go__msg__LidarState.
typedef struct unitree_go__msg__LidarState__Sequence
{
  unitree_go__msg__LidarState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__LidarState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__LIDAR_STATE__STRUCT_H_
