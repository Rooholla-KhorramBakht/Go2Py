// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/UwbSwitch.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/uwb_switch__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_go__msg__UwbSwitch__init(unitree_go__msg__UwbSwitch * msg)
{
  if (!msg) {
    return false;
  }
  // enabled
  return true;
}

void
unitree_go__msg__UwbSwitch__fini(unitree_go__msg__UwbSwitch * msg)
{
  if (!msg) {
    return;
  }
  // enabled
}

bool
unitree_go__msg__UwbSwitch__are_equal(const unitree_go__msg__UwbSwitch * lhs, const unitree_go__msg__UwbSwitch * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // enabled
  if (lhs->enabled != rhs->enabled) {
    return false;
  }
  return true;
}

bool
unitree_go__msg__UwbSwitch__copy(
  const unitree_go__msg__UwbSwitch * input,
  unitree_go__msg__UwbSwitch * output)
{
  if (!input || !output) {
    return false;
  }
  // enabled
  output->enabled = input->enabled;
  return true;
}

unitree_go__msg__UwbSwitch *
unitree_go__msg__UwbSwitch__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__UwbSwitch * msg = (unitree_go__msg__UwbSwitch *)allocator.allocate(sizeof(unitree_go__msg__UwbSwitch), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__UwbSwitch));
  bool success = unitree_go__msg__UwbSwitch__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__UwbSwitch__destroy(unitree_go__msg__UwbSwitch * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__UwbSwitch__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__UwbSwitch__Sequence__init(unitree_go__msg__UwbSwitch__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__UwbSwitch * data = NULL;

  if (size) {
    data = (unitree_go__msg__UwbSwitch *)allocator.zero_allocate(size, sizeof(unitree_go__msg__UwbSwitch), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__UwbSwitch__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__UwbSwitch__fini(&data[i - 1]);
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
unitree_go__msg__UwbSwitch__Sequence__fini(unitree_go__msg__UwbSwitch__Sequence * array)
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
      unitree_go__msg__UwbSwitch__fini(&array->data[i]);
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

unitree_go__msg__UwbSwitch__Sequence *
unitree_go__msg__UwbSwitch__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__UwbSwitch__Sequence * array = (unitree_go__msg__UwbSwitch__Sequence *)allocator.allocate(sizeof(unitree_go__msg__UwbSwitch__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__UwbSwitch__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__UwbSwitch__Sequence__destroy(unitree_go__msg__UwbSwitch__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__UwbSwitch__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__UwbSwitch__Sequence__are_equal(const unitree_go__msg__UwbSwitch__Sequence * lhs, const unitree_go__msg__UwbSwitch__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__UwbSwitch__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__UwbSwitch__Sequence__copy(
  const unitree_go__msg__UwbSwitch__Sequence * input,
  unitree_go__msg__UwbSwitch__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__UwbSwitch);
    unitree_go__msg__UwbSwitch * data =
      (unitree_go__msg__UwbSwitch *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__UwbSwitch__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__UwbSwitch__fini(&data[i]);
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
    if (!unitree_go__msg__UwbSwitch__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
