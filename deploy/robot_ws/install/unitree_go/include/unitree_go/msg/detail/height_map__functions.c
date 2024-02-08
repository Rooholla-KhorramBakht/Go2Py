// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/HeightMap.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/height_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `frame_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
unitree_go__msg__HeightMap__init(unitree_go__msg__HeightMap * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  // frame_id
  if (!rosidl_runtime_c__String__init(&msg->frame_id)) {
    unitree_go__msg__HeightMap__fini(msg);
    return false;
  }
  // resolution
  // width
  // height
  // origin
  // data
  if (!rosidl_runtime_c__float__Sequence__init(&msg->data, 0)) {
    unitree_go__msg__HeightMap__fini(msg);
    return false;
  }
  return true;
}

void
unitree_go__msg__HeightMap__fini(unitree_go__msg__HeightMap * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  // frame_id
  rosidl_runtime_c__String__fini(&msg->frame_id);
  // resolution
  // width
  // height
  // origin
  // data
  rosidl_runtime_c__float__Sequence__fini(&msg->data);
}

bool
unitree_go__msg__HeightMap__are_equal(const unitree_go__msg__HeightMap * lhs, const unitree_go__msg__HeightMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (lhs->stamp != rhs->stamp) {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame_id), &(rhs->frame_id)))
  {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // origin
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->origin[i] != rhs->origin[i]) {
      return false;
    }
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
unitree_go__msg__HeightMap__copy(
  const unitree_go__msg__HeightMap * input,
  unitree_go__msg__HeightMap * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  output->stamp = input->stamp;
  // frame_id
  if (!rosidl_runtime_c__String__copy(
      &(input->frame_id), &(output->frame_id)))
  {
    return false;
  }
  // resolution
  output->resolution = input->resolution;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // origin
  for (size_t i = 0; i < 2; ++i) {
    output->origin[i] = input->origin[i];
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

unitree_go__msg__HeightMap *
unitree_go__msg__HeightMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__HeightMap * msg = (unitree_go__msg__HeightMap *)allocator.allocate(sizeof(unitree_go__msg__HeightMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__HeightMap));
  bool success = unitree_go__msg__HeightMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__HeightMap__destroy(unitree_go__msg__HeightMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__HeightMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__HeightMap__Sequence__init(unitree_go__msg__HeightMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__HeightMap * data = NULL;

  if (size) {
    data = (unitree_go__msg__HeightMap *)allocator.zero_allocate(size, sizeof(unitree_go__msg__HeightMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__HeightMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__HeightMap__fini(&data[i - 1]);
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
unitree_go__msg__HeightMap__Sequence__fini(unitree_go__msg__HeightMap__Sequence * array)
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
      unitree_go__msg__HeightMap__fini(&array->data[i]);
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

unitree_go__msg__HeightMap__Sequence *
unitree_go__msg__HeightMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__HeightMap__Sequence * array = (unitree_go__msg__HeightMap__Sequence *)allocator.allocate(sizeof(unitree_go__msg__HeightMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__HeightMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__HeightMap__Sequence__destroy(unitree_go__msg__HeightMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__HeightMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__HeightMap__Sequence__are_equal(const unitree_go__msg__HeightMap__Sequence * lhs, const unitree_go__msg__HeightMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__HeightMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__HeightMap__Sequence__copy(
  const unitree_go__msg__HeightMap__Sequence * input,
  unitree_go__msg__HeightMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__HeightMap);
    unitree_go__msg__HeightMap * data =
      (unitree_go__msg__HeightMap *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__HeightMap__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__HeightMap__fini(&data[i]);
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
    if (!unitree_go__msg__HeightMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
