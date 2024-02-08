// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/sport_mode_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `bms_cmd`
#include "unitree_go/msg/detail/bms_cmd__functions.h"
// Member `path_point`
#include "unitree_go/msg/detail/path_point__functions.h"

bool
unitree_go__msg__SportModeCmd__init(unitree_go__msg__SportModeCmd * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // gait_type
  // speed_level
  // foot_raise_height
  // body_height
  // position
  // euler
  // velocity
  // yaw_speed
  // bms_cmd
  if (!unitree_go__msg__BmsCmd__init(&msg->bms_cmd)) {
    unitree_go__msg__SportModeCmd__fini(msg);
    return false;
  }
  // path_point
  for (size_t i = 0; i < 30; ++i) {
    if (!unitree_go__msg__PathPoint__init(&msg->path_point[i])) {
      unitree_go__msg__SportModeCmd__fini(msg);
      return false;
    }
  }
  return true;
}

void
unitree_go__msg__SportModeCmd__fini(unitree_go__msg__SportModeCmd * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // gait_type
  // speed_level
  // foot_raise_height
  // body_height
  // position
  // euler
  // velocity
  // yaw_speed
  // bms_cmd
  unitree_go__msg__BmsCmd__fini(&msg->bms_cmd);
  // path_point
  for (size_t i = 0; i < 30; ++i) {
    unitree_go__msg__PathPoint__fini(&msg->path_point[i]);
  }
}

bool
unitree_go__msg__SportModeCmd__are_equal(const unitree_go__msg__SportModeCmd * lhs, const unitree_go__msg__SportModeCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // gait_type
  if (lhs->gait_type != rhs->gait_type) {
    return false;
  }
  // speed_level
  if (lhs->speed_level != rhs->speed_level) {
    return false;
  }
  // foot_raise_height
  if (lhs->foot_raise_height != rhs->foot_raise_height) {
    return false;
  }
  // body_height
  if (lhs->body_height != rhs->body_height) {
    return false;
  }
  // position
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // euler
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->euler[i] != rhs->euler[i]) {
      return false;
    }
  }
  // velocity
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->velocity[i] != rhs->velocity[i]) {
      return false;
    }
  }
  // yaw_speed
  if (lhs->yaw_speed != rhs->yaw_speed) {
    return false;
  }
  // bms_cmd
  if (!unitree_go__msg__BmsCmd__are_equal(
      &(lhs->bms_cmd), &(rhs->bms_cmd)))
  {
    return false;
  }
  // path_point
  for (size_t i = 0; i < 30; ++i) {
    if (!unitree_go__msg__PathPoint__are_equal(
        &(lhs->path_point[i]), &(rhs->path_point[i])))
    {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__SportModeCmd__copy(
  const unitree_go__msg__SportModeCmd * input,
  unitree_go__msg__SportModeCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // gait_type
  output->gait_type = input->gait_type;
  // speed_level
  output->speed_level = input->speed_level;
  // foot_raise_height
  output->foot_raise_height = input->foot_raise_height;
  // body_height
  output->body_height = input->body_height;
  // position
  for (size_t i = 0; i < 2; ++i) {
    output->position[i] = input->position[i];
  }
  // euler
  for (size_t i = 0; i < 3; ++i) {
    output->euler[i] = input->euler[i];
  }
  // velocity
  for (size_t i = 0; i < 2; ++i) {
    output->velocity[i] = input->velocity[i];
  }
  // yaw_speed
  output->yaw_speed = input->yaw_speed;
  // bms_cmd
  if (!unitree_go__msg__BmsCmd__copy(
      &(input->bms_cmd), &(output->bms_cmd)))
  {
    return false;
  }
  // path_point
  for (size_t i = 0; i < 30; ++i) {
    if (!unitree_go__msg__PathPoint__copy(
        &(input->path_point[i]), &(output->path_point[i])))
    {
      return false;
    }
  }
  return true;
}

unitree_go__msg__SportModeCmd *
unitree_go__msg__SportModeCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__SportModeCmd * msg = (unitree_go__msg__SportModeCmd *)allocator.allocate(sizeof(unitree_go__msg__SportModeCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__SportModeCmd));
  bool success = unitree_go__msg__SportModeCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__SportModeCmd__destroy(unitree_go__msg__SportModeCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__SportModeCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__SportModeCmd__Sequence__init(unitree_go__msg__SportModeCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__SportModeCmd * data = NULL;

  if (size) {
    data = (unitree_go__msg__SportModeCmd *)allocator.zero_allocate(size, sizeof(unitree_go__msg__SportModeCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__SportModeCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__SportModeCmd__fini(&data[i - 1]);
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
unitree_go__msg__SportModeCmd__Sequence__fini(unitree_go__msg__SportModeCmd__Sequence * array)
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
      unitree_go__msg__SportModeCmd__fini(&array->data[i]);
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

unitree_go__msg__SportModeCmd__Sequence *
unitree_go__msg__SportModeCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__SportModeCmd__Sequence * array = (unitree_go__msg__SportModeCmd__Sequence *)allocator.allocate(sizeof(unitree_go__msg__SportModeCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__SportModeCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__SportModeCmd__Sequence__destroy(unitree_go__msg__SportModeCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__SportModeCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__SportModeCmd__Sequence__are_equal(const unitree_go__msg__SportModeCmd__Sequence * lhs, const unitree_go__msg__SportModeCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__SportModeCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__SportModeCmd__Sequence__copy(
  const unitree_go__msg__SportModeCmd__Sequence * input,
  unitree_go__msg__SportModeCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__SportModeCmd);
    unitree_go__msg__SportModeCmd * data =
      (unitree_go__msg__SportModeCmd *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__SportModeCmd__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__SportModeCmd__fini(&data[i]);
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
    if (!unitree_go__msg__SportModeCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
