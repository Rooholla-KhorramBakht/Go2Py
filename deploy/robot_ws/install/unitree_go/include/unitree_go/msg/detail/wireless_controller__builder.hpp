// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/WirelessController.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__BUILDER_HPP_

#include "unitree_go/msg/detail/wireless_controller__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_WirelessController_keys
{
public:
  explicit Init_WirelessController_keys(::unitree_go::msg::WirelessController & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::WirelessController keys(::unitree_go::msg::WirelessController::_keys_type arg)
  {
    msg_.keys = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::WirelessController msg_;
};

class Init_WirelessController_ry
{
public:
  explicit Init_WirelessController_ry(::unitree_go::msg::WirelessController & msg)
  : msg_(msg)
  {}
  Init_WirelessController_keys ry(::unitree_go::msg::WirelessController::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_WirelessController_keys(msg_);
  }

private:
  ::unitree_go::msg::WirelessController msg_;
};

class Init_WirelessController_rx
{
public:
  explicit Init_WirelessController_rx(::unitree_go::msg::WirelessController & msg)
  : msg_(msg)
  {}
  Init_WirelessController_ry rx(::unitree_go::msg::WirelessController::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_WirelessController_ry(msg_);
  }

private:
  ::unitree_go::msg::WirelessController msg_;
};

class Init_WirelessController_ly
{
public:
  explicit Init_WirelessController_ly(::unitree_go::msg::WirelessController & msg)
  : msg_(msg)
  {}
  Init_WirelessController_rx ly(::unitree_go::msg::WirelessController::_ly_type arg)
  {
    msg_.ly = std::move(arg);
    return Init_WirelessController_rx(msg_);
  }

private:
  ::unitree_go::msg::WirelessController msg_;
};

class Init_WirelessController_lx
{
public:
  Init_WirelessController_lx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WirelessController_ly lx(::unitree_go::msg::WirelessController::_lx_type arg)
  {
    msg_.lx = std::move(arg);
    return Init_WirelessController_ly(msg_);
  }

private:
  ::unitree_go::msg::WirelessController msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::WirelessController>()
{
  return unitree_go::msg::builder::Init_WirelessController_lx();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__BUILDER_HPP_
