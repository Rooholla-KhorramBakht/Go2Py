// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from unitree_api:msg/Request.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "unitree_api/msg/detail/request__rosidl_typesupport_introspection_c.h"
#include "unitree_api/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "unitree_api/msg/detail/request__functions.h"
#include "unitree_api/msg/detail/request__struct.h"


// Include directives for member types
// Member `header`
#include "unitree_api/msg/request_header.h"
// Member `header`
#include "unitree_api/msg/detail/request_header__rosidl_typesupport_introspection_c.h"
// Member `parameter`
#include "rosidl_runtime_c/string_functions.h"
// Member `binary`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Request__rosidl_typesupport_introspection_c__Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  unitree_api__msg__Request__init(message_memory);
}

void Request__rosidl_typesupport_introspection_c__Request_fini_function(void * message_memory)
{
  unitree_api__msg__Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Request__rosidl_typesupport_introspection_c__Request_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__Request, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parameter",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__Request, parameter),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "binary",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api__msg__Request, binary),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Request__rosidl_typesupport_introspection_c__Request_message_members = {
  "unitree_api__msg",  // message namespace
  "Request",  // message name
  3,  // number of fields
  sizeof(unitree_api__msg__Request),
  Request__rosidl_typesupport_introspection_c__Request_message_member_array,  // message members
  Request__rosidl_typesupport_introspection_c__Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Request__rosidl_typesupport_introspection_c__Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Request__rosidl_typesupport_introspection_c__Request_message_type_support_handle = {
  0,
  &Request__rosidl_typesupport_introspection_c__Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_unitree_api
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, Request)() {
  Request__rosidl_typesupport_introspection_c__Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unitree_api, msg, RequestHeader)();
  if (!Request__rosidl_typesupport_introspection_c__Request_message_type_support_handle.typesupport_identifier) {
    Request__rosidl_typesupport_introspection_c__Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Request__rosidl_typesupport_introspection_c__Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
