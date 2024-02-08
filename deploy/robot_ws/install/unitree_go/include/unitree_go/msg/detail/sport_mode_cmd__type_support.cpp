// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_go/msg/detail/sport_mode_cmd__struct.hpp"
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

void SportModeCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_go::msg::SportModeCmd(_init);
}

void SportModeCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_go::msg::SportModeCmd *>(message_memory);
  typed_message->~SportModeCmd();
}

size_t size_function__SportModeCmd__position(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__SportModeCmd__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__SportModeCmd__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__SportModeCmd__euler(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__SportModeCmd__euler(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__SportModeCmd__euler(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__SportModeCmd__velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__SportModeCmd__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__SportModeCmd__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__SportModeCmd__path_point(const void * untyped_member)
{
  (void)untyped_member;
  return 30;
}

const void * get_const_function__SportModeCmd__path_point(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<unitree_go::msg::PathPoint, 30> *>(untyped_member);
  return &member[index];
}

void * get_function__SportModeCmd__path_point(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<unitree_go::msg::PathPoint, 30> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SportModeCmd_message_member_array[11] = {
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gait_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, gait_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed_level",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, speed_level),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "foot_raise_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, foot_raise_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "body_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, body_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SportModeCmd__position,  // size() function pointer
    get_const_function__SportModeCmd__position,  // get_const(index) function pointer
    get_function__SportModeCmd__position,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "euler",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, euler),  // bytes offset in struct
    nullptr,  // default value
    size_function__SportModeCmd__euler,  // size() function pointer
    get_const_function__SportModeCmd__euler,  // get_const(index) function pointer
    get_function__SportModeCmd__euler,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__SportModeCmd__velocity,  // size() function pointer
    get_const_function__SportModeCmd__velocity,  // get_const(index) function pointer
    get_function__SportModeCmd__velocity,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, yaw_speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bms_cmd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_go::msg::BmsCmd>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, bms_cmd),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "path_point",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_go::msg::PathPoint>(),  // members of sub message
    true,  // is array
    30,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::SportModeCmd, path_point),  // bytes offset in struct
    nullptr,  // default value
    size_function__SportModeCmd__path_point,  // size() function pointer
    get_const_function__SportModeCmd__path_point,  // get_const(index) function pointer
    get_function__SportModeCmd__path_point,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SportModeCmd_message_members = {
  "unitree_go::msg",  // message namespace
  "SportModeCmd",  // message name
  11,  // number of fields
  sizeof(unitree_go::msg::SportModeCmd),
  SportModeCmd_message_member_array,  // message members
  SportModeCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  SportModeCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SportModeCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SportModeCmd_message_members,
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
get_message_type_support_handle<unitree_go::msg::SportModeCmd>()
{
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::SportModeCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_go, msg, SportModeCmd)() {
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::SportModeCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
