// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_api:msg/RequestIdentity.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_api/msg/detail/request_identity__rosidl_typesupport_introspection_c.h"
#include "unitree_api/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_api/msg/detail/request_identity__functions.h"
#include "unitree_api/msg/detail/request_identity__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_api__msg__RequestIdentity__init(message_memory);
}

void RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_fini_function(void * message_memory)
{
  unitree_api__msg__RequestIdentity__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestIdentity, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "api_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestIdentity, api_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_members = {
  "unitree_api__msg",  // message namespace
  "RequestIdentity",  // message name
  2,  // number of fields
  sizeof(unitree_api__msg__RequestIdentity),
  RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_member_array,  // message members
  RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_init_function,  // function to initialize message memory (memory has to be allocated)
  RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_type_support_handle = {
  0,
  &RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_api
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestIdentity)() {
  if (!RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_type_support_handle.typesupport_identifier) {
    RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RequestIdentity__rosidl_typesupport_introspection_c__RequestIdentity_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
