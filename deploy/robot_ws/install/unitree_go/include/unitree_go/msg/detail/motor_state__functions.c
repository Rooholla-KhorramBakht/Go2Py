// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/MotorState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/motor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_go__msg__MotorState__init(unitree_go__msg__MotorState * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // q
  // dq
  // ddq
  // tau_est
  // q_raw
  // dq_raw
  // ddq_raw
  // temperature
  // lost
  // reserve
  return true;
}

void
unitree_go__msg__MotorState__fini(unitree_go__msg__MotorState * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // q
  // dq
  // ddq
  // tau_est
  // q_raw
  // dq_raw
  // ddq_raw
  // temperature
  // lost
  // reserve
}

bool
unitree_go__msg__MotorState__are_equal(const unitree_go__msg__MotorState * lhs, const unitree_go__msg__MotorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // q
  if (lhs->q != rhs->q) {
    return false;
  }
  // dq
  if (lhs->dq != rhs->dq) {
    return false;
  }
  // ddq
  if (lhs->ddq != rhs->ddq) {
    return false;
  }
  // tau_est
  if (lhs->tau_est != rhs->tau_est) {
    return false;
  }
  // q_raw
  if (lhs->q_raw != rhs->q_raw) {
    return false;
  }
  // dq_raw
  if (lhs->dq_raw != rhs->dq_raw) {
    return false;
  }
  // ddq_raw
  if (lhs->ddq_raw != rhs->ddq_raw) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // lost
  if (lhs->lost != rhs->lost) {
    return false;
  }
  // reserve
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->reserve[i] != rhs->reserve[i]) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__MotorState__copy(
  const unitree_go__msg__MotorState * input,
  unitree_go__msg__MotorState * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // q
  output->q = input->q;
  // dq
  output->dq = input->dq;
  // ddq
  output->ddq = input->ddq;
  // tau_est
  output->tau_est = input->tau_est;
  // q_raw
  output->q_raw = input->q_raw;
  // dq_raw
  output->dq_raw = input->dq_raw;
  // ddq_raw
  output->ddq_raw = input->ddq_raw;
  // temperature
  output->temperature = input->temperature;
  // lost
  output->lost = input->lost;
  // reserve
  for (size_t i = 0; i < 2; ++i) {
    output->reserve[i] = input->reserve[i];
  }
  return true;
}

unitree_go__msg__MotorState *
unitree_go__msg__MotorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorState * msg = (unitree_go__msg__MotorState *)allocator.allocate(sizeof(unitree_go__msg__MotorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__MotorState));
  bool success = unitree_go__msg__MotorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__MotorState__destroy(unitree_go__msg__MotorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__MotorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__MotorState__Sequence__init(unitree_go__msg__MotorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorState * data = NULL;

  if (size) {
    data = (unitree_go__msg__MotorState *)allocator.zero_allocate(size, sizeof(unitree_go__msg__MotorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__MotorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__MotorState__fini(&data[i - 1]);
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
unitree_go__msg__MotorState__Sequence__fini(unitree_go__msg__MotorState__Sequence * array)
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
      unitree_go__msg__MotorState__fini(&array->data[i]);
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

unitree_go__msg__MotorState__Sequence *
unitree_go__msg__MotorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorState__Sequence * array = (unitree_go__msg__MotorState__Sequence *)allocator.allocate(sizeof(unitree_go__msg__MotorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__MotorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__MotorState__Sequence__destroy(unitree_go__msg__MotorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__MotorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__MotorState__Sequence__are_equal(const unitree_go__msg__MotorState__Sequence * lhs, const unitree_go__msg__MotorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__MotorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__MotorState__Sequence__copy(
  const unitree_go__msg__MotorState__Sequence * input,
  unitree_go__msg__MotorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__MotorState);
    unitree_go__msg__MotorState * data =
      (unitree_go__msg__MotorState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__MotorState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__MotorState__fini(&data[i]);
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
    if (!unitree_go__msg__MotorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
