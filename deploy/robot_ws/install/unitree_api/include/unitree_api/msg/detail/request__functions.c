// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_api:msg/Request.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "unitree_api/msg/detail/request_header__functions.h"
// Member `parameter`
#include "rosidl_runtime_c/string_functions.h"
// Member `binary`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
unitree_api__msg__Request__init(unitree_api__msg__Request * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!unitree_api__msg__RequestHeader__init(&msg->header)) {
    unitree_api__msg__Request__fini(msg);
    return false;
  }
  // parameter
  if (!rosidl_runtime_c__String__init(&msg->parameter)) {
    unitree_api__msg__Request__fini(msg);
    return false;
  }
  // binary
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->binary, 0)) {
    unitree_api__msg__Request__fini(msg);
    return false;
  }
  return true;
}

void
unitree_api__msg__Request__fini(unitree_api__msg__Request * msg)
{
  if (!msg) {
    return;
  }
  // header
  unitree_api__msg__RequestHeader__fini(&msg->header);
  // parameter
  rosidl_runtime_c__String__fini(&msg->parameter);
  // binary
  rosidl_runtime_c__uint8__Sequence__fini(&msg->binary);
}

bool
unitree_api__msg__Request__are_equal(const unitree_api__msg__Request * lhs, const unitree_api__msg__Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!unitree_api__msg__RequestHeader__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // parameter
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->parameter), &(rhs->parameter)))
  {
    return false;
  }
  // binary
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->binary), &(rhs->binary)))
  {
    return false;
  }
  return true;
}

bool
unitree_api__msg__Request__copy(
  const unitree_api__msg__Request * input,
  unitree_api__msg__Request * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!unitree_api__msg__RequestHeader__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // parameter
  if (!rosidl_runtime_c__String__copy(
      &(input->parameter), &(output->parameter)))
  {
    return false;
  }
  // binary
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->binary), &(output->binary)))
  {
    return false;
  }
  return true;
}

unitree_api__msg__Request *
unitree_api__msg__Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__Request * msg = (unitree_api__msg__Request *)allocator.allocate(sizeof(unitree_api__msg__Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_api__msg__Request));
  bool success = unitree_api__msg__Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_api__msg__Request__destroy(unitree_api__msg__Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_api__msg__Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_api__msg__Request__Sequence__init(unitree_api__msg__Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__Request * data = NULL;

  if (size) {
    data = (unitree_api__msg__Request *)allocator.zero_allocate(size, sizeof(unitree_api__msg__Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_api__msg__Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_api__msg__Request__fini(&data[i - 1]);
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
unitree_api__msg__Request__Sequence__fini(unitree_api__msg__Request__Sequence * array)
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
      unitree_api__msg__Request__fini(&array->data[i]);
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

unitree_api__msg__Request__Sequence *
unitree_api__msg__Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_api__msg__Request__Sequence * array = (unitree_api__msg__Request__Sequence *)allocator.allocate(sizeof(unitree_api__msg__Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_api__msg__Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_api__msg__Request__Sequence__destroy(unitree_api__msg__Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_api__msg__Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_api__msg__Request__Sequence__are_equal(const unitree_api__msg__Request__Sequence * lhs, const unitree_api__msg__Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_api__msg__Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_api__msg__Request__Sequence__copy(
  const unitree_api__msg__Request__Sequence * input,
  unitree_api__msg__Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_api__msg__Request);
    unitree_api__msg__Request * data =
      (unitree_api__msg__Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_api__msg__Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_api__msg__Request__fini(&data[i]);
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
    if (!unitree_api__msg__Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
