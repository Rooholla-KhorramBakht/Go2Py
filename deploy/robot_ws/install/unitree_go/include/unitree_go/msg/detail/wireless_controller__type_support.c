// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_go:msg/WirelessController.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_go/msg/detail/wireless_controller__rosidl_typesupport_introspection_c.h"
#include "unitree_go/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_go/msg/detail/wireless_controller__functions.h"
#include "unitree_go/msg/detail/wireless_controller__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void WirelessController__rosidl_typesupport_introspection_c__WirelessController_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_go__msg__WirelessController__init(message_memory);
}

void WirelessController__rosidl_typesupport_introspection_c__WirelessController_fini_function(void * message_memory)
{
  unitree_go__msg__WirelessController__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_member_array[5] = {
  {
    "lx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__WirelessController, lx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ly",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__WirelessController, ly),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__WirelessController, rx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ry",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__WirelessController, ry),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keys",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go__msg__WirelessController, keys),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_members = {
  "unitree_go__msg",  // message namespace
  "WirelessController",  // message name
  5,  // number of fields
  sizeof(unitree_go__msg__WirelessController),
  WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_member_array,  // message members
  WirelessController__rosidl_typesupport_introspection_c__WirelessController_init_function,  // function to initialize message memory (memory has to be allocated)
  WirelessController__rosidl_typesupport_introspection_c__WirelessController_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_type_support_handle = {
  0,
  &WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_go
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_go, msg, WirelessController)() {
  if (!WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_type_support_handle.typesupport_identifier) {
    WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &WirelessController__rosidl_typesupport_introspection_c__WirelessController_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
