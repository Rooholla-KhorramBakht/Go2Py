// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/IMUState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_go/msg/detail/imu_state__struct.hpp"
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

void IMUState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_go::msg::IMUState(_init);
}

void IMUState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_go::msg::IMUState *>(message_memory);
  typed_message->~IMUState();
}

size_t size_function__IMUState__quaternion(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__IMUState__quaternion(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUState__quaternion(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 4> *>(untyped_member);
  return &member[index];
}

size_t size_function__IMUState__gyroscope(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUState__gyroscope(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUState__gyroscope(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__IMUState__accelerometer(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUState__accelerometer(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUState__accelerometer(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__IMUState__rpy(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUState__rpy(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUState__rpy(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember IMUState_message_member_array[5] = {
  {
    "quaternion",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::IMUState, quaternion),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUState__quaternion,  // size() function pointer
    get_const_function__IMUState__quaternion,  // get_const(index) function pointer
    get_function__IMUState__quaternion,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gyroscope",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::IMUState, gyroscope),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUState__gyroscope,  // size() function pointer
    get_const_function__IMUState__gyroscope,  // get_const(index) function pointer
    get_function__IMUState__gyroscope,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "accelerometer",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::IMUState, accelerometer),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUState__accelerometer,  // size() function pointer
    get_const_function__IMUState__accelerometer,  // get_const(index) function pointer
    get_function__IMUState__accelerometer,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rpy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::IMUState, rpy),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUState__rpy,  // size() function pointer
    get_const_function__IMUState__rpy,  // get_const(index) function pointer
    get_function__IMUState__rpy,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_go::msg::IMUState, temperature),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers IMUState_message_members = {
  "unitree_go::msg",  // message namespace
  "IMUState",  // message name
  5,  // number of fields
  sizeof(unitree_go::msg::IMUState),
  IMUState_message_member_array,  // message members
  IMUState_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t IMUState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &IMUState_message_members,
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
get_message_type_support_handle<unitree_go::msg::IMUState>()
{
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::IMUState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_go, msg, IMUState)() {
  return &::unitree_go::msg::rosidl_typesupport_introspection_cpp::IMUState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
