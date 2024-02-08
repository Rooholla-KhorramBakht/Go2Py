// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_api:msg/ResponseHeader.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_api/msg/detail/response_header__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace unitree_api
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ResponseHeader_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_api::msg::ResponseHeader(_init);
}

void ResponseHeader_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_api::msg::ResponseHeader *>(message_memory);
  typed_message->~ResponseHeader();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ResponseHeader_message_member_array[2] = {
  {
    "identity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_api::msg::RequestIdentity>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api::msg::ResponseHeader, identity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_api::msg::ResponseStatus>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_api::msg::ResponseHeader, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ResponseHeader_message_members = {
  "unitree_api::msg",  // message namespace
  "ResponseHeader",  // message name
  2,  // number of fields
  sizeof(unitree_api::msg::ResponseHeader),
  ResponseHeader_message_member_array,  // message members
  ResponseHeader_init_function,  // function to initialize message memory (memory has to be allocated)
  ResponseHeader_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ResponseHeader_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ResponseHeader_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace unitree_api


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_api::msg::ResponseHeader>()
{
  return &::unitree_api::msg::rosidl_typesupport_introspection_cpp::ResponseHeader_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_api, msg, ResponseHeader)() {
  return &::unitree_api::msg::rosidl_typesupport_introspection_cpp::ResponseHeader_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
