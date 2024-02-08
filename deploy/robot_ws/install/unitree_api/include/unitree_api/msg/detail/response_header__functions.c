// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_api:msg/ResponseHeader.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/response_header__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `identity`
#include "unitree_api/msg/detail/request_identity__functions.h"
// Member `status`
#include "unitree_api/msg/detail/response_status__functions.h"

bool
unitree_api__msg__ResponseHeader__init(unitree_api__msg__ResponseHeader * msg)
{
  if (!msg) {
    return false;
  }
  // identity
  if (!unitree_api__msg__RequestIdentity__init(&msg->identity)) {
    unitree_api__msg__ResponseHeader__fini(msg);
    return false;
  }
  // status
  if (!unitree_api__msg__ResponseStatus__init(&msg->status)) {
    unitree_api__msg__ResponseHeader__fini(msg);
    return false;
  }
  return true;
}

void
unitree_api__msg__ResponseHeader__fini(unitree_api__msg__ResponseHeader * msg)
{
  if (!msg) {
    return;
  }
  // identity
  unitree_api__msg__RequestIdentity__fini(&msg->identity);
  // status
  unitree_api__msg__ResponseStatus__fini(&msg->status);
}

bool
unitree_api__msg__ResponseHeader__are_equal(const unitree_api__msg__ResponseHeader * lhs, const unitree_api__msg__ResponseHeader * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // identity
  if (!unitree_api__msg__RequestIdentity__are_equal(
      &(lhs->identity), &(rhs->identity)))
  {
    return false;
  }
  // status
  if (!unitree_api__msg__ResponseStatus__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
unitree_api__msg__ResponseHeader__copy(
  const unitree_api__msg__ResponseHeader * input,
  unitree_api__msg__ResponseHeader * output)
{
  if (!input || !output) {
    return false;
  }
  // identity
  if (!unitree_api__msg__RequestIdentity__copy(
      &(input->identity), &(output->identity)))
  {
    return false;
  }
  // status
  if (!unitree_api__msg__ResponseStatus__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

unitree_api__msg__ResponseHeader *
unitree_api__msg__ResponseHeader__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__ResponseHeader * msg = (unitree_api__msg__ResponseHeader *)allocator.allocate(sizeof(unitree_api__msg__ResponseHeader), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_api__msg__ResponseHeader));
  bool success = unitree_api__msg__ResponseHeader__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_api__msg__ResponseHeader__destroy(unitree_api__msg__ResponseHeader * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_api__msg__ResponseHeader__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_api__msg__ResponseHeader__Sequence__init(unitree_api__msg__ResponseHeader__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__ResponseHeader * data = NULL;

  if (size) {
    data = (unitree_api__msg__ResponseHeader *)allocator.zero_allocate(size, sizeof(unitree_api__msg__ResponseHeader), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_api__msg__ResponseHeader__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_api__msg__ResponseHeader__fini(&data[i - 1]);
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
unitree_api__msg__ResponseHeader__Sequence__fini(unitree_api__msg__ResponseHeader__Sequence * array)
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
      unitree_api__msg__ResponseHeader__fini(&array->data[i]);
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

unitree_api__msg__ResponseHeader__Sequence *
unitree_api__msg__ResponseHeader__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__ResponseHeader__Sequence * array = (unitree_api__msg__ResponseHeader__Sequence *)allocator.allocate(sizeof(unitree_api__msg__ResponseHeader__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_api__msg__ResponseHeader__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_api__msg__ResponseHeader__Sequence__destroy(unitree_api__msg__ResponseHeader__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_api__msg__ResponseHeader__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_api__msg__ResponseHeader__Sequence__are_equal(const unitree_api__msg__ResponseHeader__Sequence * lhs, const unitree_api__msg__ResponseHeader__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_api__msg__ResponseHeader__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_api__msg__ResponseHeader__Sequence__copy(
  const unitree_api__msg__ResponseHeader__Sequence * input,
  unitree_api__msg__ResponseHeader__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_api__msg__ResponseHeader);
    unitree_api__msg__ResponseHeader * data =
      (unitree_api__msg__ResponseHeader *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_api__msg__ResponseHeader__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_api__msg__ResponseHeader__fini(&data[i]);
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
    if (!unitree_api__msg__ResponseHeader__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
