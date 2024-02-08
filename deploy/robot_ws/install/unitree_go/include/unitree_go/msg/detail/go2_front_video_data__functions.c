// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/go2_front_video_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `video720p`
// Member `video360p`
// Member `video180p`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
unitree_go__msg__Go2FrontVideoData__init(unitree_go__msg__Go2FrontVideoData * msg)
{
  if (!msg) {
    return false;
  }
  // time_frame
  // video720p
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->video720p, 0)) {
    unitree_go__msg__Go2FrontVideoData__fini(msg);
    return false;
  }
  // video360p
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->video360p, 0)) {
    unitree_go__msg__Go2FrontVideoData__fini(msg);
    return false;
  }
  // video180p
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->video180p, 0)) {
    unitree_go__msg__Go2FrontVideoData__fini(msg);
    return false;
  }
  return true;
}

void
unitree_go__msg__Go2FrontVideoData__fini(unitree_go__msg__Go2FrontVideoData * msg)
{
  if (!msg) {
    return;
  }
  // time_frame
  // video720p
  rosidl_runtime_c__uint8__Sequence__fini(&msg->video720p);
  // video360p
  rosidl_runtime_c__uint8__Sequence__fini(&msg->video360p);
  // video180p
  rosidl_runtime_c__uint8__Sequence__fini(&msg->video180p);
}

bool
unitree_go__msg__Go2FrontVideoData__are_equal(const unitree_go__msg__Go2FrontVideoData * lhs, const unitree_go__msg__Go2FrontVideoData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // time_frame
  if (lhs->time_frame != rhs->time_frame) {
    return false;
  }
  // video720p
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->video720p), &(rhs->video720p)))
  {
    return false;
  }
  // video360p
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->video360p), &(rhs->video360p)))
  {
    return false;
  }
  // video180p
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->video180p), &(rhs->video180p)))
  {
    return false;
  }
  return true;
}

bool
unitree_go__msg__Go2FrontVideoData__copy(
  const unitree_go__msg__Go2FrontVideoData * input,
  unitree_go__msg__Go2FrontVideoData * output)
{
  if (!input || !output) {
    return false;
  }
  // time_frame
  output->time_frame = input->time_frame;
  // video720p
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->video720p), &(output->video720p)))
  {
    return false;
  }
  // video360p
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->video360p), &(output->video360p)))
  {
    return false;
  }
  // video180p
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->video180p), &(output->video180p)))
  {
    return false;
  }
  return true;
}

unitree_go__msg__Go2FrontVideoData *
unitree_go__msg__Go2FrontVideoData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__Go2FrontVideoData * msg = (unitree_go__msg__Go2FrontVideoData *)allocator.allocate(sizeof(unitree_go__msg__Go2FrontVideoData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__Go2FrontVideoData));
  bool success = unitree_go__msg__Go2FrontVideoData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__Go2FrontVideoData__destroy(unitree_go__msg__Go2FrontVideoData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__Go2FrontVideoData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__Go2FrontVideoData__Sequence__init(unitree_go__msg__Go2FrontVideoData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__Go2FrontVideoData * data = NULL;

  if (size) {
    data = (unitree_go__msg__Go2FrontVideoData *)allocator.zero_allocate(size, sizeof(unitree_go__msg__Go2FrontVideoData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__Go2FrontVideoData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__Go2FrontVideoData__fini(&data[i - 1]);
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
unitree_go__msg__Go2FrontVideoData__Sequence__fini(unitree_go__msg__Go2FrontVideoData__Sequence * array)
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
      unitree_go__msg__Go2FrontVideoData__fini(&array->data[i]);
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

unitree_go__msg__Go2FrontVideoData__Sequence *
unitree_go__msg__Go2FrontVideoData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__Go2FrontVideoData__Sequence * array = (unitree_go__msg__Go2FrontVideoData__Sequence *)allocator.allocate(sizeof(unitree_go__msg__Go2FrontVideoData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__Go2FrontVideoData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__Go2FrontVideoData__Sequence__destroy(unitree_go__msg__Go2FrontVideoData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__Go2FrontVideoData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__Go2FrontVideoData__Sequence__are_equal(const unitree_go__msg__Go2FrontVideoData__Sequence * lhs, const unitree_go__msg__Go2FrontVideoData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__Go2FrontVideoData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__Go2FrontVideoData__Sequence__copy(
  const unitree_go__msg__Go2FrontVideoData__Sequence * input,
  unitree_go__msg__Go2FrontVideoData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__Go2FrontVideoData);
    unitree_go__msg__Go2FrontVideoData * data =
      (unitree_go__msg__Go2FrontVideoData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__Go2FrontVideoData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__Go2FrontVideoData__fini(&data[i]);
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
    if (!unitree_go__msg__Go2FrontVideoData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
