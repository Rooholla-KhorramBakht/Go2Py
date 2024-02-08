// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/UwbSwitch.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__UWB_SWITCH__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__UWB_SWITCH__BUILDER_HPP_

#include "unitree_go/msg/detail/uwb_switch__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_UwbSwitch_enabled
{
public:
  Init_UwbSwitch_enabled()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::unitree_go::msg::UwbSwitch enabled(::unitree_go::msg::UwbSwitch::_enabled_type arg)
  {
    msg_.enabled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::UwbSwitch msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::UwbSwitch>()
{
  return unitree_go::msg::builder::Init_UwbSwitch_enabled();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__UWB_SWITCH__BUILDER_HPP_
