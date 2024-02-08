// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__BUILDER_HPP_

#include "unitree_go/msg/detail/sport_mode_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_SportModeCmd_path_point
{
public:
  explicit Init_SportModeCmd_path_point(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::SportModeCmd path_point(::unitree_go::msg::SportModeCmd::_path_point_type arg)
  {
    msg_.path_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_bms_cmd
{
public:
  explicit Init_SportModeCmd_bms_cmd(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_path_point bms_cmd(::unitree_go::msg::SportModeCmd::_bms_cmd_type arg)
  {
    msg_.bms_cmd = std::move(arg);
    return Init_SportModeCmd_path_point(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_yaw_speed
{
public:
  explicit Init_SportModeCmd_yaw_speed(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_bms_cmd yaw_speed(::unitree_go::msg::SportModeCmd::_yaw_speed_type arg)
  {
    msg_.yaw_speed = std::move(arg);
    return Init_SportModeCmd_bms_cmd(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_velocity
{
public:
  explicit Init_SportModeCmd_velocity(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_yaw_speed velocity(::unitree_go::msg::SportModeCmd::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_SportModeCmd_yaw_speed(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_euler
{
public:
  explicit Init_SportModeCmd_euler(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_velocity euler(::unitree_go::msg::SportModeCmd::_euler_type arg)
  {
    msg_.euler = std::move(arg);
    return Init_SportModeCmd_velocity(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_position
{
public:
  explicit Init_SportModeCmd_position(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_euler position(::unitree_go::msg::SportModeCmd::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_SportModeCmd_euler(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_body_height
{
public:
  explicit Init_SportModeCmd_body_height(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_position body_height(::unitree_go::msg::SportModeCmd::_body_height_type arg)
  {
    msg_.body_height = std::move(arg);
    return Init_SportModeCmd_position(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_foot_raise_height
{
public:
  explicit Init_SportModeCmd_foot_raise_height(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_body_height foot_raise_height(::unitree_go::msg::SportModeCmd::_foot_raise_height_type arg)
  {
    msg_.foot_raise_height = std::move(arg);
    return Init_SportModeCmd_body_height(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_speed_level
{
public:
  explicit Init_SportModeCmd_speed_level(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_foot_raise_height speed_level(::unitree_go::msg::SportModeCmd::_speed_level_type arg)
  {
    msg_.speed_level = std::move(arg);
    return Init_SportModeCmd_foot_raise_height(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_gait_type
{
public:
  explicit Init_SportModeCmd_gait_type(::unitree_go::msg::SportModeCmd & msg)
  : msg_(msg)
  {}
  Init_SportModeCmd_speed_level gait_type(::unitree_go::msg::SportModeCmd::_gait_type_type arg)
  {
    msg_.gait_type = std::move(arg);
    return Init_SportModeCmd_speed_level(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

class Init_SportModeCmd_mode
{
public:
  Init_SportModeCmd_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SportModeCmd_gait_type mode(::unitree_go::msg::SportModeCmd::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_SportModeCmd_gait_type(msg_);
  }

private:
  ::unitree_go::msg::SportModeCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::SportModeCmd>()
{
  return unitree_go::msg::builder::Init_SportModeCmd_mode();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__BUILDER_HPP_
