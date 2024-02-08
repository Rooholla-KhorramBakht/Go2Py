// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/TimeSpec.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_go/msg/detail/time_spec__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace unitree_go
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TimeSpec_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_go::msg::TimeSpec(_init);
}

void TimeSpec_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_go::msg::TimeSpec *>(message_memory);
  typed_message->~TimeSpec();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TimeSpec_message_member_array[2] = {
  {
    "sec",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::TimeSpec, sec),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "nanosec",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::TimeSpec, nanosec),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TimeSpec_message_members = {
  "unitree_go::msg",  // message namespace
  "TimeSpec",  // message name
  2,  // number of fields
  sizeof(unitree_go::msg::TimeSpec),
  TimeSpec_message_member_array,  // message members
  TimeSpec_init_function,  // function to initialize message memory (memory has to be allocated)
  TimeSpec_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TimeSpec_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TimeSpec_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace unitree_go


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_go::msg::TimeSpec>()
{
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::TimeSpec_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_go, msg, TimeSpec)() {
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::TimeSpec_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
