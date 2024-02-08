// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_api:msg/RequestLease.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/request_lease__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_api__msg__RequestLease__init(unitree_api__msg__RequestLease * msg)
{
  if (!msg) {
    return false;
  }
  // id
  return true;
}

void
unitree_api__msg__RequestLease__fini(unitree_api__msg__RequestLease * msg)
{
  if (!msg) {
    return;
  }
  // id
}

bool
unitree_api__msg__RequestLease__are_equal(const unitree_api__msg__RequestLease * lhs, const unitree_api__msg__RequestLease * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  return true;
}

bool
unitree_api__msg__RequestLease__copy(
  const unitree_api__msg__RequestLease * input,
  unitree_api__msg__RequestLease * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  return true;
}

unitree_api__msg__RequestLease *
unitree_api__msg__RequestLease__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__RequestLease * msg = (unitree_api__msg__RequestLease *)allocator.allocate(sizeof(unitree_api__msg__RequestLease), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_api__msg__RequestLease));
  bool success = unitree_api__msg__RequestLease__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_api__msg__RequestLease__destroy(unitree_api__msg__RequestLease * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_api__msg__RequestLease__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_api__msg__RequestLease__Sequence__init(unitree_api__msg__RequestLease__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__RequestLease * data = NULL;

  if (size) {
    data = (unitree_api__msg__RequestLease *)allocator.zero_allocate(size, sizeof(unitree_api__msg__RequestLease), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_api__msg__RequestLease__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_api__msg__RequestLease__fini(&data[i - 1]);
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
unitree_api__msg__RequestLease__Sequence__fini(unitree_api__msg__RequestLease__Sequence * array)
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
      unitree_api__msg__RequestLease__fini(&array->data[i]);
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

unitree_api__msg__RequestLease__Sequence *
unitree_api__msg__RequestLease__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__RequestLease__Sequence * array = (unitree_api__msg__RequestLease__Sequence *)allocator.allocate(sizeof(unitree_api__msg__RequestLease__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_api__msg__RequestLease__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_api__msg__RequestLease__Sequence__destroy(unitree_api__msg__RequestLease__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_api__msg__RequestLease__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_api__msg__RequestLease__Sequence__are_equal(const unitree_api__msg__RequestLease__Sequence * lhs, const unitree_api__msg__RequestLease__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_api__msg__RequestLease__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_api__msg__RequestLease__Sequence__copy(
  const unitree_api__msg__RequestLease__Sequence * input,
  unitree_api__msg__RequestLease__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_api__msg__RequestLease);
    unitree_api__msg__RequestLease * data =
      (unitree_api__msg__RequestLease *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_api__msg__RequestLease__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_api__msg__RequestLease__fini(&data[i]);
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
    if (!unitree_api__msg__RequestLease__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
