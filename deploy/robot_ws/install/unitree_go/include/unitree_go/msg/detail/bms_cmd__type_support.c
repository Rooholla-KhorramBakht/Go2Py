// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_go:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_go/msg/detail/bms_cmd__rosidl_typesupport_introspection_c.h"
#include "unitree_go/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_go/msg/detail/bms_cmd__functions.h"
#include "unitree_go/msg/detail/bms_cmd__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_go__msg__BmsCmd__init(message_memory);
}

void BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_fini_function(void * message_memory)
{
  unitree_go__msg__BmsCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_member_array[2] = {
  {
    "off",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__BmsCmd, off),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reserve",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__BmsCmd, reserve),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_members = {
  "unitree_go__msg",  // message namespace
  "BmsCmd",  // message name
  2,  // number of fields
  sizeof(unitree_go__msg__BmsCmd),
  BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_member_array,  // message members
  BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_type_support_handle = {
  0,
  &BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_go
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_go, msg, BmsCmd)() {
  if (!BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_type_support_handle.typesupport_identifier) {
    BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BmsCmd__rosidl_typesupport_introspection_c__BmsCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
