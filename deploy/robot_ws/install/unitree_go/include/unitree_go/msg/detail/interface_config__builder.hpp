// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/InterfaceConfig.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__INTERFACE_CONFIG__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__INTERFACE_CONFIG__BUILDER_HPP_

#include "unitree_go/msg/detail/interface_config__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_InterfaceConfig_reserve
{
public:
  explicit Init_InterfaceConfig_reserve(::unitree_go::msg::InterfaceConfig & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::InterfaceConfig reserve(::unitree_go::msg::InterfaceConfig::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::InterfaceConfig msg_;
};

class Init_InterfaceConfig_value
{
public:
  explicit Init_InterfaceConfig_value(::unitree_go::msg::InterfaceConfig & msg)
  : msg_(msg)
  {}
  Init_InterfaceConfig_reserve value(::unitree_go::msg::InterfaceConfig::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_InterfaceConfig_reserve(msg_);
  }

private:
  ::unitree_go::msg::InterfaceConfig msg_;
};

class Init_InterfaceConfig_mode
{
public:
  Init_InterfaceConfig_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InterfaceConfig_value mode(::unitree_go::msg::InterfaceConfig::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_InterfaceConfig_value(msg_);
  }

private:
  ::unitree_go::msg::InterfaceConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::InterfaceConfig>()
{
  return unitree_go::msg::builder::Init_InterfaceConfig_mode();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__INTERFACE_CONFIG__BUILDER_HPP_
