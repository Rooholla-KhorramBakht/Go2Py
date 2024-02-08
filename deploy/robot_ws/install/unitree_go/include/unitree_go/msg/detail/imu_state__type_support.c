// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_go:msg/IMUState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_go/msg/detail/imu_state__rosidl_typesupport_introspection_c.h"
#include "unitree_go/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_go/msg/detail/imu_state__functions.h"
#include "unitree_go/msg/detail/imu_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void IMUState__rosidl_typesupport_introspection_c__IMUState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_go__msg__IMUState__init(message_memory);
}

void IMUState__rosidl_typesupport_introspection_c__IMUState_fini_function(void * message_memory)
{
  unitree_go__msg__IMUState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IMUState__rosidl_typesupport_introspection_c__IMUState_message_member_array[5] = {
  {
    "quaternion",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__IMUState, quaternion),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyroscope",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__IMUState, gyroscope),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accelerometer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__IMUState, accelerometer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rpy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__IMUState, rpy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__IMUState, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IMUState__rosidl_typesupport_introspection_c__IMUState_message_members = {
  "unitree_go__msg",  // message namespace
  "IMUState",  // message name
  5,  // number of fields
  sizeof(unitree_go__msg__IMUState),
  IMUState__rosidl_typesupport_introspection_c__IMUState_message_member_array,  // message members
  IMUState__rosidl_typesupport_introspection_c__IMUState_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUState__rosidl_typesupport_introspection_c__IMUState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IMUState__rosidl_typesupport_introspection_c__IMUState_message_type_support_handle = {
  0,
  &IMUState__rosidl_typesupport_introspection_c__IMUState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_go
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_go, msg, IMUState)() {
  if (!IMUState__rosidl_typesupport_introspection_c__IMUState_message_type_support_handle.typesupport_identifier) {
    IMUState__rosidl_typesupport_introspection_c__IMUState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IMUState__rosidl_typesupport_introspection_c__IMUState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
