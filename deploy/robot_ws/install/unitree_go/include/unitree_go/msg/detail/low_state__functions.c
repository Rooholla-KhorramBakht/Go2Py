// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/LowState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/low_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `imu_state`
#include "unitree_go/msg/detail/imu_state__functions.h"
// Member `motor_state`
#include "unitree_go/msg/detail/motor_state__functions.h"
// Member `bms_state`
#include "unitree_go/msg/detail/bms_state__functions.h"

bool
unitree_go__msg__LowState__init(unitree_go__msg__LowState * msg)
{
  if (!msg) {
    return false;
  }
  // head
  // level_flag
  // frame_reserve
  // sn
  // version
  // bandwidth
  // imu_state
  if (!unitree_go__msg__IMUState__init(&msg->imu_state)) {
    unitree_go__msg__LowState__fini(msg);
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 20; ++i) {
    if (!unitree_go__msg__MotorState__init(&msg->motor_state[i])) {
      unitree_go__msg__LowState__fini(msg);
      return false;
    }
  }
  // bms_state
  if (!unitree_go__msg__BmsState__init(&msg->bms_state)) {
    unitree_go__msg__LowState__fini(msg);
    return false;
  }
  // foot_force
  // foot_force_est
  // tick
  // wireless_remote
  // bit_flag
  // adc_reel
  // temperature_ntc1
  // temperature_ntc2
  // power_v
  // power_a
  // fan_frequency
  // reserve
  // crc
  return true;
}

void
unitree_go__msg__LowState__fini(unitree_go__msg__LowState * msg)
{
  if (!msg) {
    return;
  }
  // head
  // level_flag
  // frame_reserve
  // sn
  // version
  // bandwidth
  // imu_state
  unitree_go__msg__IMUState__fini(&msg->imu_state);
  // motor_state
  for (size_t i = 0; i < 20; ++i) {
    unitree_go__msg__MotorState__fini(&msg->motor_state[i]);
  }
  // bms_state
  unitree_go__msg__BmsState__fini(&msg->bms_state);
  // foot_force
  // foot_force_est
  // tick
  // wireless_remote
  // bit_flag
  // adc_reel
  // temperature_ntc1
  // temperature_ntc2
  // power_v
  // power_a
  // fan_frequency
  // reserve
  // crc
}

bool
unitree_go__msg__LowState__are_equal(const unitree_go__msg__LowState * lhs, const unitree_go__msg__LowState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // head
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->head[i] != rhs->head[i]) {
      return false;
    }
  }
  // level_flag
  if (lhs->level_flag != rhs->level_flag) {
    return false;
  }
  // frame_reserve
  if (lhs->frame_reserve != rhs->frame_reserve) {
    return false;
  }
  // sn
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->sn[i] != rhs->sn[i]) {
      return false;
    }
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->version[i] != rhs->version[i]) {
      return false;
    }
  }
  // bandwidth
  if (lhs->bandwidth != rhs->bandwidth) {
    return false;
  }
  // imu_state
  if (!unitree_go__msg__IMUState__are_equal(
      &(lhs->imu_state), &(rhs->imu_state)))
  {
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 20; ++i) {
    if (!unitree_go__msg__MotorState__are_equal(
        &(lhs->motor_state[i]), &(rhs->motor_state[i])))
    {
      return false;
    }
  }
  // bms_state
  if (!unitree_go__msg__BmsState__are_equal(
      &(lhs->bms_state), &(rhs->bms_state)))
  {
    return false;
  }
  // foot_force
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->foot_force[i] != rhs->foot_force[i]) {
      return false;
    }
  }
  // foot_force_est
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->foot_force_est[i] != rhs->foot_force_est[i]) {
      return false;
    }
  }
  // tick
  if (lhs->tick != rhs->tick) {
    return false;
  }
  // wireless_remote
  for (size_t i = 0; i < 40; ++i) {
    if (lhs->wireless_remote[i] != rhs->wireless_remote[i]) {
      return false;
    }
  }
  // bit_flag
  if (lhs->bit_flag != rhs->bit_flag) {
    return false;
  }
  // adc_reel
  if (lhs->adc_reel != rhs->adc_reel) {
    return false;
  }
  // temperature_ntc1
  if (lhs->temperature_ntc1 != rhs->temperature_ntc1) {
    return false;
  }
  // temperature_ntc2
  if (lhs->temperature_ntc2 != rhs->temperature_ntc2) {
    return false;
  }
  // power_v
  if (lhs->power_v != rhs->power_v) {
    return false;
  }
  // power_a
  if (lhs->power_a != rhs->power_a) {
    return false;
  }
  // fan_frequency
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->fan_frequency[i] != rhs->fan_frequency[i]) {
      return false;
    }
  }
  // reserve
  if (lhs->reserve != rhs->reserve) {
    return false;
  }
  // crc
  if (lhs->crc != rhs->crc) {
    return false;
  }
  return true;
}

bool
unitree_go__msg__LowState__copy(
  const unitree_go__msg__LowState * input,
  unitree_go__msg__LowState * output)
{
  if (!input || !output) {
    return false;
  }
  // head
  for (size_t i = 0; i < 2; ++i) {
    output->head[i] = input->head[i];
  }
  // level_flag
  output->level_flag = input->level_flag;
  // frame_reserve
  output->frame_reserve = input->frame_reserve;
  // sn
  for (size_t i = 0; i < 2; ++i) {
    output->sn[i] = input->sn[i];
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    output->version[i] = input->version[i];
  }
  // bandwidth
  output->bandwidth = input->bandwidth;
  // imu_state
  if (!unitree_go__msg__IMUState__copy(
      &(input->imu_state), &(output->imu_state)))
  {
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 20; ++i) {
    if (!unitree_go__msg__MotorState__copy(
        &(input->motor_state[i]), &(output->motor_state[i])))
    {
      return false;
    }
  }
  // bms_state
  if (!unitree_go__msg__BmsState__copy(
      &(input->bms_state), &(output->bms_state)))
  {
    return false;
  }
  // foot_force
  for (size_t i = 0; i < 4; ++i) {
    output->foot_force[i] = input->foot_force[i];
  }
  // foot_force_est
  for (size_t i = 0; i < 4; ++i) {
    output->foot_force_est[i] = input->foot_force_est[i];
  }
  // tick
  output->tick = input->tick;
  // wireless_remote
  for (size_t i = 0; i < 40; ++i) {
    output->wireless_remote[i] = input->wireless_remote[i];
  }
  // bit_flag
  output->bit_flag = input->bit_flag;
  // adc_reel
  output->adc_reel = input->adc_reel;
  // temperature_ntc1
  output->temperature_ntc1 = input->temperature_ntc1;
  // temperature_ntc2
  output->temperature_ntc2 = input->temperature_ntc2;
  // power_v
  output->power_v = input->power_v;
  // power_a
  output->power_a = input->power_a;
  // fan_frequency
  for (size_t i = 0; i < 4; ++i) {
    output->fan_frequency[i] = input->fan_frequency[i];
  }
  // reserve
  output->reserve = input->reserve;
  // crc
  output->crc = input->crc;
  return true;
}

unitree_go__msg__LowState *
unitree_go__msg__LowState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__LowState * msg = (unitree_go__msg__LowState *)allocator.allocate(sizeof(unitree_go__msg__LowState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__LowState));
  bool success = unitree_go__msg__LowState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__LowState__destroy(unitree_go__msg__LowState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__LowState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__LowState__Sequence__init(unitree_go__msg__LowState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__LowState * data = NULL;

  if (size) {
    data = (unitree_go__msg__LowState *)allocator.zero_allocate(size, sizeof(unitree_go__msg__LowState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__LowState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__LowState__fini(&data[i - 1]);
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
unitree_go__msg__LowState__Sequence__fini(unitree_go__msg__LowState__Sequence * array)
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
      unitree_go__msg__LowState__fini(&array->data[i]);
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

unitree_go__msg__LowState__Sequence *
unitree_go__msg__LowState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__LowState__Sequence * array = (unitree_go__msg__LowState__Sequence *)allocator.allocate(sizeof(unitree_go__msg__LowState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__LowState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__LowState__Sequence__destroy(unitree_go__msg__LowState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__LowState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__LowState__Sequence__are_equal(const unitree_go__msg__LowState__Sequence * lhs, const unitree_go__msg__LowState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__LowState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__LowState__Sequence__copy(
  const unitree_go__msg__LowState__Sequence * input,
  unitree_go__msg__LowState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__LowState);
    unitree_go__msg__LowState * data =
      (unitree_go__msg__LowState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__LowState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__LowState__fini(&data[i]);
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
    if (!unitree_go__msg__LowState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
