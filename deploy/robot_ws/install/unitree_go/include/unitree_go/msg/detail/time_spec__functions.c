// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/TimeSpec.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/time_spec__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_go__msg__TimeSpec__init(unitree_go__msg__TimeSpec * msg)
{
  if (!msg) {
    return false;
  }
  // sec
  // nanosec
  return true;
}

void
unitree_go__msg__TimeSpec__fini(unitree_go__msg__TimeSpec * msg)
{
  if (!msg) {
    return;
  }
  // sec
  // nanosec
}

bool
unitree_go__msg__TimeSpec__are_equal(const unitree_go__msg__TimeSpec * lhs, const unitree_go__msg__TimeSpec * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sec
  if (lhs->sec != rhs->sec) {
    return false;
  }
  // nanosec
  if (lhs->nanosec != rhs->nanosec) {
    return false;
  }
  return true;
}

bool
unitree_go__msg__TimeSpec__copy(
  const unitree_go__msg__TimeSpec * input,
  unitree_go__msg__TimeSpec * output)
{
  if (!input || !output) {
    return false;
  }
  // sec
  output->sec = input->sec;
  // nanosec
  output->nanosec = input->nanosec;
  return true;
}

unitree_go__msg__TimeSpec *
unitree_go__msg__TimeSpec__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__TimeSpec * msg = (unitree_go__msg__TimeSpec *)allocator.allocate(sizeof(unitree_go__msg__TimeSpec), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__TimeSpec));
  bool success = unitree_go__msg__TimeSpec__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__TimeSpec__destroy(unitree_go__msg__TimeSpec * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__TimeSpec__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__TimeSpec__Sequence__init(unitree_go__msg__TimeSpec__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__TimeSpec * data = NULL;

  if (size) {
    data = (unitree_go__msg__TimeSpec *)allocator.zero_allocate(size, sizeof(unitree_go__msg__TimeSpec), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__TimeSpec__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__TimeSpec__fini(&data[i - 1]);
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
unitree_go__msg__TimeSpec__Sequence__fini(unitree_go__msg__TimeSpec__Sequence * array)
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
      unitree_go__msg__TimeSpec__fini(&array->data[i]);
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

unitree_go__msg__TimeSpec__Sequence *
unitree_go__msg__TimeSpec__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__TimeSpec__Sequence * array = (unitree_go__msg__TimeSpec__Sequence *)allocator.allocate(sizeof(unitree_go__msg__TimeSpec__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__TimeSpec__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__TimeSpec__Sequence__destroy(unitree_go__msg__TimeSpec__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__TimeSpec__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__TimeSpec__Sequence__are_equal(const unitree_go__msg__TimeSpec__Sequence * lhs, const unitree_go__msg__TimeSpec__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__TimeSpec__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__TimeSpec__Sequence__copy(
  const unitree_go__msg__TimeSpec__Sequence * input,
  unitree_go__msg__TimeSpec__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__TimeSpec);
    unitree_go__msg__TimeSpec * data =
      (unitree_go__msg__TimeSpec *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__TimeSpec__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__TimeSpec__fini(&data[i]);
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
    if (!unitree_go__msg__TimeSpec__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
