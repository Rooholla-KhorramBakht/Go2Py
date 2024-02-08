// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/LidarState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/lidar_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `firmware_version`
// Member `software_version`
// Member `sdk_version`
#include "rosidl_runtime_c/string_functions.h"

bool
unitree_go__msg__LidarState__init(unitree_go__msg__LidarState * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  // firmware_version
  if (!rosidl_runtime_c__String__init(&msg->firmware_version)) {
    unitree_go__msg__LidarState__fini(msg);
    return false;
  }
  // software_version
  if (!rosidl_runtime_c__String__init(&msg->software_version)) {
    unitree_go__msg__LidarState__fini(msg);
    return false;
  }
  // sdk_version
  if (!rosidl_runtime_c__String__init(&msg->sdk_version)) {
    unitree_go__msg__LidarState__fini(msg);
    return false;
  }
  // sys_rotation_speed
  // com_rotation_speed
  // error_state
  // cloud_frequency
  // cloud_packet_loss_rate
  // cloud_size
  // cloud_scan_num
  // imu_frequency
  // imu_packet_loss_rate
  // imu_rpy
  // serial_recv_stamp
  // serial_buffer_size
  // serial_buffer_read
  return true;
}

void
unitree_go__msg__LidarState__fini(unitree_go__msg__LidarState * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  // firmware_version
  rosidl_runtime_c__String__fini(&msg->firmware_version);
  // software_version
  rosidl_runtime_c__String__fini(&msg->software_version);
  // sdk_version
  rosidl_runtime_c__String__fini(&msg->sdk_version);
  // sys_rotation_speed
  // com_rotation_speed
  // error_state
  // cloud_frequency
  // cloud_packet_loss_rate
  // cloud_size
  // cloud_scan_num
  // imu_frequency
  // imu_packet_loss_rate
  // imu_rpy
  // serial_recv_stamp
  // serial_buffer_size
  // serial_buffer_read
}

bool
unitree_go__msg__LidarState__are_equal(const unitree_go__msg__LidarState * lhs, const unitree_go__msg__LidarState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (lhs->stamp != rhs->stamp) {
    return false;
  }
  // firmware_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->firmware_version), &(rhs->firmware_version)))
  {
    return false;
  }
  // software_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->software_version), &(rhs->software_version)))
  {
    return false;
  }
  // sdk_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sdk_version), &(rhs->sdk_version)))
  {
    return false;
  }
  // sys_rotation_speed
  if (lhs->sys_rotation_speed != rhs->sys_rotation_speed) {
    return false;
  }
  // com_rotation_speed
  if (lhs->com_rotation_speed != rhs->com_rotation_speed) {
    return false;
  }
  // error_state
  if (lhs->error_state != rhs->error_state) {
    return false;
  }
  // cloud_frequency
  if (lhs->cloud_frequency != rhs->cloud_frequency) {
    return false;
  }
  // cloud_packet_loss_rate
  if (lhs->cloud_packet_loss_rate != rhs->cloud_packet_loss_rate) {
    return false;
  }
  // cloud_size
  if (lhs->cloud_size != rhs->cloud_size) {
    return false;
  }
  // cloud_scan_num
  if (lhs->cloud_scan_num != rhs->cloud_scan_num) {
    return false;
  }
  // imu_frequency
  if (lhs->imu_frequency != rhs->imu_frequency) {
    return false;
  }
  // imu_packet_loss_rate
  if (lhs->imu_packet_loss_rate != rhs->imu_packet_loss_rate) {
    return false;
  }
  // imu_rpy
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->imu_rpy[i] != rhs->imu_rpy[i]) {
      return false;
    }
  }
  // serial_recv_stamp
  if (lhs->serial_recv_stamp != rhs->serial_recv_stamp) {
    return false;
  }
  // serial_buffer_size
  if (lhs->serial_buffer_size != rhs->serial_buffer_size) {
    return false;
  }
  // serial_buffer_read
  if (lhs->serial_buffer_read != rhs->serial_buffer_read) {
    return false;
  }
  return true;
}

bool
unitree_go__msg__LidarState__copy(
  const unitree_go__msg__LidarState * input,
  unitree_go__msg__LidarState * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  output->stamp = input->stamp;
  // firmware_version
  if (!rosidl_runtime_c__String__copy(
      &(input->firmware_version), &(output->firmware_version)))
  {
    return false;
  }
  // software_version
  if (!rosidl_runtime_c__String__copy(
      &(input->software_version), &(output->software_version)))
  {
    return false;
  }
  // sdk_version
  if (!rosidl_runtime_c__String__copy(
      &(input->sdk_version), &(output->sdk_version)))
  {
    return false;
  }
  // sys_rotation_speed
  output->sys_rotation_speed = input->sys_rotation_speed;
  // com_rotation_speed
  output->com_rotation_speed = input->com_rotation_speed;
  // error_state
  output->error_state = input->error_state;
  // cloud_frequency
  output->cloud_frequency = input->cloud_frequency;
  // cloud_packet_loss_rate
  output->cloud_packet_loss_rate = input->cloud_packet_loss_rate;
  // cloud_size
  output->cloud_size = input->cloud_size;
  // cloud_scan_num
  output->cloud_scan_num = input->cloud_scan_num;
  // imu_frequency
  output->imu_frequency = input->imu_frequency;
  // imu_packet_loss_rate
  output->imu_packet_loss_rate = input->imu_packet_loss_rate;
  // imu_rpy
  for (size_t i = 0; i < 3; ++i) {
    output->imu_rpy[i] = input->imu_rpy[i];
  }
  // serial_recv_stamp
  output->serial_recv_stamp = input->serial_recv_stamp;
  // serial_buffer_size
  output->serial_buffer_size = input->serial_buffer_size;
  // serial_buffer_read
  output->serial_buffer_read = input->serial_buffer_read;
  return true;
}

unitree_go__msg__LidarState *
unitree_go__msg__LidarState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__LidarState * msg = (unitree_go__msg__LidarState *)allocator.allocate(sizeof(unitree_go__msg__LidarState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__LidarState));
  bool success = unitree_go__msg__LidarState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__LidarState__destroy(unitree_go__msg__LidarState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__LidarState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__LidarState__Sequence__init(unitree_go__msg__LidarState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__LidarState * data = NULL;

  if (size) {
    data = (unitree_go__msg__LidarState *)allocator.zero_allocate(size, sizeof(unitree_go__msg__LidarState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__LidarState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__LidarState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
unitree_go__msg__LidarState__Sequence__fini(unitree_go__msg__LidarState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      unitree_go__msg__LidarState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

unitree_go__msg__LidarState__Sequence *
unitree_go__msg__LidarState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__LidarState__Sequence * array = (unitree_go__msg__LidarState__Sequence *)allocator.allocate(sizeof(unitree_go__msg__LidarState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__LidarState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__LidarState__Sequence__destroy(unitree_go__msg__LidarState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__LidarState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__LidarState__Sequence__are_equal(const unitree_go__msg__LidarState__Sequence * lhs, const unitree_go__msg__LidarState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__LidarState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__LidarState__Sequence__copy(
  const unitree_go__msg__LidarState__Sequence * input,
  unitree_go__msg__LidarState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__LidarState);
    unitree_go__msg__LidarState * data =
      (unitree_go__msg__LidarState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__LidarState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__LidarState__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!unitree_go__msg__LidarState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
