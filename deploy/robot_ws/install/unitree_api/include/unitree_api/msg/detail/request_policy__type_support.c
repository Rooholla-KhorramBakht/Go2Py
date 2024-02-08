// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_api:msg/RequestPolicy.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_api/msg/detail/request_policy__rosidl_typesupport_introspection_c.h"
#include "unitree_api/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_api/msg/detail/request_policy__functions.h"
#include "unitree_api/msg/detail/request_policy__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_api__msg__RequestPolicy__init(message_memory);
}

void RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_fini_function(void * message_memory)
{
  unitree_api__msg__RequestPolicy__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_member_array[2] = {
  {
    "priority",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestPolicy, priority),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "noreply",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestPolicy, noreply),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_members = {
  "unitree_api__msg",  // message namespace
  "RequestPolicy",  // message name
  2,  // number of fields
  sizeof(unitree_api__msg__RequestPolicy),
  RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_member_array,  // message members
  RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_init_function,  // function to initialize message memory (memory has to be allocated)
  RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_type_support_handle = {
  0,
  &RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_api
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestPolicy)() {
  if (!RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_type_support_handle.typesupport_identifier) {
    RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RequestPolicy__rosidl_typesupport_introspection_c__RequestPolicy_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
