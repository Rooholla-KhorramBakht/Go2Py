// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_api/msg/detail/request_header__rosidl_typesupport_introspection_c.h"
#include "unitree_api/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_api/msg/detail/request_header__functions.h"
#include "unitree_api/msg/detail/request_header__struct.h"


// Include directives for member types
// Member `identity`
#include "unitree_api/msg/request_identity.h"
// Member `identity`
#include "unitree_api/msg/detail/request_identity__rosidl_typesupport_introspection_c.h"
// Member `lease`
#include "unitree_api/msg/request_lease.h"
// Member `lease`
#include "unitree_api/msg/detail/request_lease__rosidl_typesupport_introspection_c.h"
// Member `policy`
#include "unitree_api/msg/request_policy.h"
// Member `policy`
#include "unitree_api/msg/detail/request_policy__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_api__msg__RequestHeader__init(message_memory);
}

void RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_fini_function(void * message_memory)
{
  unitree_api__msg__RequestHeader__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_member_array[3] = {
  {
    "identity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestHeader, identity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lease",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestHeader, lease),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "policy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__RequestHeader, policy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_members = {
  "unitree_api__msg",  // message namespace
  "RequestHeader",  // message name
  3,  // number of fields
  sizeof(unitree_api__msg__RequestHeader),
  RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_member_array,  // message members
  RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_init_function,  // function to initialize message memory (memory has to be allocated)
  RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_type_support_handle = {
  0,
  &RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_api
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestHeader)() {
  RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestIdentity)();
  RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestLease)();
  RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestPolicy)();
  if (!RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_type_support_handle.typesupport_identifier) {
    RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RequestHeader__rosidl_typesupport_introspection_c__RequestHeader_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
