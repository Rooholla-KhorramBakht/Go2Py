// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/LowCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_go/msg/detail/low_cmd__struct.hpp"
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

void LowCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_go::msg::LowCmd(_init);
}

void LowCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_go::msg::LowCmd *>(message_memory);
  typed_message->~LowCmd();
}

size_t size_function__LowCmd__head(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__LowCmd__head(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__head(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowCmd__sn(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__LowCmd__sn(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__sn(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowCmd__version(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__LowCmd__version(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__version(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowCmd__motor_cmd(const void * untyped_member)
{
  (void)untyped_member;
  return 20;
}

const void * get_const_function__LowCmd__motor_cmd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<unitree_go::msg::MotorCmd, 20> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__motor_cmd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<unitree_go::msg::MotorCmd, 20> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowCmd__wireless_remote(const void * untyped_member)
{
  (void)untyped_member;
  return 40;
}

const void * get_const_function__LowCmd__wireless_remote(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 40> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__wireless_remote(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 40> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowCmd__led(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__LowCmd__led(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__led(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowCmd__fan(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__LowCmd__fan(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__LowCmd__fan(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 2> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LowCmd_message_member_array[14] = {
  {
    "head",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, head),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__head,  // size() function pointer
    get_const_function__LowCmd__head,  // get_const(index) function pointer
    get_function__LowCmd__head,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "level_flag",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, level_flag),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frame_reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, frame_reserve),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sn",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, sn),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__sn,  // size() function pointer
    get_const_function__LowCmd__sn,  // get_const(index) function pointer
    get_function__LowCmd__sn,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "version",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, version),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__version,  // size() function pointer
    get_const_function__LowCmd__version,  // get_const(index) function pointer
    get_function__LowCmd__version,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bandwidth",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, bandwidth),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_cmd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_go::msg::MotorCmd>(),  // members of sub message
    true,  // is array
    20,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, motor_cmd),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__motor_cmd,  // size() function pointer
    get_const_function__LowCmd__motor_cmd,  // get_const(index) function pointer
    get_function__LowCmd__motor_cmd,  // get(index) function pointer
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
    offsetof(unitree_go::msg::LowCmd, bms_cmd),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wireless_remote",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    40,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, wireless_remote),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__wireless_remote,  // size() function pointer
    get_const_function__LowCmd__wireless_remote,  // get_const(index) function pointer
    get_function__LowCmd__wireless_remote,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "led",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, led),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__led,  // size() function pointer
    get_const_function__LowCmd__led,  // get_const(index) function pointer
    get_function__LowCmd__led,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fan",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, fan),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowCmd__fan,  // size() function pointer
    get_const_function__LowCmd__fan,  // get_const(index) function pointer
    get_function__LowCmd__fan,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gpio",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, gpio),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, reserve),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "crc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::LowCmd, crc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LowCmd_message_members = {
  "unitree_go::msg",  // message namespace
  "LowCmd",  // message name
  14,  // number of fields
  sizeof(unitree_go::msg::LowCmd),
  LowCmd_message_member_array,  // message members
  LowCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  LowCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LowCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LowCmd_message_members,
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
get_message_type_support_handle<unitree_go::msg::LowCmd>()
{
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::LowCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_go, msg, LowCmd)() {
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::LowCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
