// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_CMD__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__LOW_CMD__BUILDER_HPP_

#include "unitree_go/msg/detail/low_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_LowCmd_crc
{
public:
  explicit Init_LowCmd_crc(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::LowCmd crc(::unitree_go::msg::LowCmd::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_reserve
{
public:
  explicit Init_LowCmd_reserve(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_crc reserve(::unitree_go::msg::LowCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_LowCmd_crc(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_gpio
{
public:
  explicit Init_LowCmd_gpio(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_reserve gpio(::unitree_go::msg::LowCmd::_gpio_type arg)
  {
    msg_.gpio = std::move(arg);
    return Init_LowCmd_reserve(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_fan
{
public:
  explicit Init_LowCmd_fan(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_gpio fan(::unitree_go::msg::LowCmd::_fan_type arg)
  {
    msg_.fan = std::move(arg);
    return Init_LowCmd_gpio(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_led
{
public:
  explicit Init_LowCmd_led(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_fan led(::unitree_go::msg::LowCmd::_led_type arg)
  {
    msg_.led = std::move(arg);
    return Init_LowCmd_fan(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_wireless_remote
{
public:
  explicit Init_LowCmd_wireless_remote(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_led wireless_remote(::unitree_go::msg::LowCmd::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_LowCmd_led(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_bms_cmd
{
public:
  explicit Init_LowCmd_bms_cmd(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_wireless_remote bms_cmd(::unitree_go::msg::LowCmd::_bms_cmd_type arg)
  {
    msg_.bms_cmd = std::move(arg);
    return Init_LowCmd_wireless_remote(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_motor_cmd
{
public:
  explicit Init_LowCmd_motor_cmd(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_bms_cmd motor_cmd(::unitree_go::msg::LowCmd::_motor_cmd_type arg)
  {
    msg_.motor_cmd = std::move(arg);
    return Init_LowCmd_bms_cmd(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_bandwidth
{
public:
  explicit Init_LowCmd_bandwidth(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_motor_cmd bandwidth(::unitree_go::msg::LowCmd::_bandwidth_type arg)
  {
    msg_.bandwidth = std::move(arg);
    return Init_LowCmd_motor_cmd(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_version
{
public:
  explicit Init_LowCmd_version(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_bandwidth version(::unitree_go::msg::LowCmd::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_LowCmd_bandwidth(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_sn
{
public:
  explicit Init_LowCmd_sn(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_version sn(::unitree_go::msg::LowCmd::_sn_type arg)
  {
    msg_.sn = std::move(arg);
    return Init_LowCmd_version(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_frame_reserve
{
public:
  explicit Init_LowCmd_frame_reserve(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_sn frame_reserve(::unitree_go::msg::LowCmd::_frame_reserve_type arg)
  {
    msg_.frame_reserve = std::move(arg);
    return Init_LowCmd_sn(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_level_flag
{
public:
  explicit Init_LowCmd_level_flag(::unitree_go::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_frame_reserve level_flag(::unitree_go::msg::LowCmd::_level_flag_type arg)
  {
    msg_.level_flag = std::move(arg);
    return Init_LowCmd_frame_reserve(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

class Init_LowCmd_head
{
public:
  Init_LowCmd_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowCmd_level_flag head(::unitree_go::msg::LowCmd::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_LowCmd_level_flag(msg_);
  }

private:
  ::unitree_go::msg::LowCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::LowCmd>()
{
  return unitree_go::msg::builder::Init_LowCmd_head();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__LOW_CMD__BUILDER_HPP_
