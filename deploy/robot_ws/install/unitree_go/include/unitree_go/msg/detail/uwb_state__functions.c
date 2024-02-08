// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/UwbState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/uwb_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_go__msg__UwbState__init(unitree_go__msg__UwbState * msg)
{
  if (!msg) {
    return false;
  }
  // version
  // channel
  // joy_mode
  // orientation_est
  // pitch_est
  // distance_est
  // yaw_est
  // tag_roll
  // tag_pitch
  // tag_yaw
  // base_roll
  // base_pitch
  // base_yaw
  // joystick
  // error_state
  // buttons
  // enabled_from_app
  return true;
}

void
unitree_go__msg__UwbState__fini(unitree_go__msg__UwbState * msg)
{
  if (!msg) {
    return;
  }
  // version
  // channel
  // joy_mode
  // orientation_est
  // pitch_est
  // distance_est
  // yaw_est
  // tag_roll
  // tag_pitch
  // tag_yaw
  // base_roll
  // base_pitch
  // base_yaw
  // joystick
  // error_state
  // buttons
  // enabled_from_app
}

bool
unitree_go__msg__UwbState__are_equal(const unitree_go__msg__UwbState * lhs, const unitree_go__msg__UwbState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->version[i] != rhs->version[i]) {
      return false;
    }
  }
  // channel
  if (lhs->channel != rhs->channel) {
    return false;
  }
  // joy_mode
  if (lhs->joy_mode != rhs->joy_mode) {
    return false;
  }
  // orientation_est
  if (lhs->orientation_est != rhs->orientation_est) {
    return false;
  }
  // pitch_est
  if (lhs->pitch_est != rhs->pitch_est) {
    return false;
  }
  // distance_est
  if (lhs->distance_est != rhs->distance_est) {
    return false;
  }
  // yaw_est
  if (lhs->yaw_est != rhs->yaw_est) {
    return false;
  }
  // tag_roll
  if (lhs->tag_roll != rhs->tag_roll) {
    return false;
  }
  // tag_pitch
  if (lhs->tag_pitch != rhs->tag_pitch) {
    return false;
  }
  // tag_yaw
  if (lhs->tag_yaw != rhs->tag_yaw) {
    return false;
  }
  // base_roll
  if (lhs->base_roll != rhs->base_roll) {
    return false;
  }
  // base_pitch
  if (lhs->base_pitch != rhs->base_pitch) {
    return false;
  }
  // base_yaw
  if (lhs->base_yaw != rhs->base_yaw) {
    return false;
  }
  // joystick
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->joystick[i] != rhs->joystick[i]) {
      return false;
    }
  }
  // error_state
  if (lhs->error_state != rhs->error_state) {
    return false;
  }
  // buttons
  if (lhs->buttons != rhs->buttons) {
    return false;
  }
  // enabled_from_app
  if (lhs->enabled_from_app != rhs->enabled_from_app) {
    return false;
  }
  return true;
}

bool
unitree_go__msg__UwbState__copy(
  const unitree_go__msg__UwbState * input,
  unitree_go__msg__UwbState * output)
{
  if (!input || !output) {
    return false;
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    output->version[i] = input->version[i];
  }
  // channel
  output->channel = input->channel;
  // joy_mode
  output->joy_mode = input->joy_mode;
  // orientation_est
  output->orientation_est = input->orientation_est;
  // pitch_est
  output->pitch_est = input->pitch_est;
  // distance_est
  output->distance_est = input->distance_est;
  // yaw_est
  output->yaw_est = input->yaw_est;
  // tag_roll
  output->tag_roll = input->tag_roll;
  // tag_pitch
  output->tag_pitch = input->tag_pitch;
  // tag_yaw
  output->tag_yaw = input->tag_yaw;
  // base_roll
  output->base_roll = input->base_roll;
  // base_pitch
  output->base_pitch = input->base_pitch;
  // base_yaw
  output->base_yaw = input->base_yaw;
  // joystick
  for (size_t i = 0; i < 2; ++i) {
    output->joystick[i] = input->joystick[i];
  }
  // error_state
  output->error_state = input->error_state;
  // buttons
  output->buttons = input->buttons;
  // enabled_from_app
  output->enabled_from_app = input->enabled_from_app;
  return true;
}

unitree_go__msg__UwbState *
unitree_go__msg__UwbState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__UwbState * msg = (unitree_go__msg__UwbState *)allocator.allocate(sizeof(unitree_go__msg__UwbState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__UwbState));
  bool success = unitree_go__msg__UwbState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__UwbState__destroy(unitree_go__msg__UwbState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__UwbState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__UwbState__Sequence__init(unitree_go__msg__UwbState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__UwbState * data = NULL;

  if (size) {
    data = (unitree_go__msg__UwbState *)allocator.zero_allocate(size, sizeof(unitree_go__msg__UwbState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__UwbState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__UwbState__fini(&data[i - 1]);
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
unitree_go__msg__UwbState__Sequence__fini(unitree_go__msg__UwbState__Sequence * array)
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
      unitree_go__msg__UwbState__fini(&array->data[i]);
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

unitree_go__msg__UwbState__Sequence *
unitree_go__msg__UwbState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__UwbState__Sequence * array = (unitree_go__msg__UwbState__Sequence *)allocator.allocate(sizeof(unitree_go__msg__UwbState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__UwbState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__UwbState__Sequence__destroy(unitree_go__msg__UwbState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__UwbState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__UwbState__Sequence__are_equal(const unitree_go__msg__UwbState__Sequence * lhs, const unitree_go__msg__UwbState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__UwbState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__UwbState__Sequence__copy(
  const unitree_go__msg__UwbState__Sequence * input,
  unitree_go__msg__UwbState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__UwbState);
    unitree_go__msg__UwbState * data =
      (unitree_go__msg__UwbState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__UwbState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__UwbState__fini(&data[i]);
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
    if (!unitree_go__msg__UwbState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
