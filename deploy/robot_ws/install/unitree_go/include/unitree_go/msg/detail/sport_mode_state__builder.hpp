// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/SportModeState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__BUILDER_HPP_

#include "unitree_go/msg/detail/sport_mode_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_SportModeState_foot_speed_body
{
public:
  explicit Init_SportModeState_foot_speed_body(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::SportModeState foot_speed_body(::unitree_go::msg::SportModeState::_foot_speed_body_type arg)
  {
    msg_.foot_speed_body = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_foot_position_body
{
public:
  explicit Init_SportModeState_foot_position_body(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_foot_speed_body foot_position_body(::unitree_go::msg::SportModeState::_foot_position_body_type arg)
  {
    msg_.foot_position_body = std::move(arg);
    return Init_SportModeState_foot_speed_body(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_foot_force
{
public:
  explicit Init_SportModeState_foot_force(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_foot_position_body foot_force(::unitree_go::msg::SportModeState::_foot_force_type arg)
  {
    msg_.foot_force = std::move(arg);
    return Init_SportModeState_foot_position_body(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_range_obstacle
{
public:
  explicit Init_SportModeState_range_obstacle(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_foot_force range_obstacle(::unitree_go::msg::SportModeState::_range_obstacle_type arg)
  {
    msg_.range_obstacle = std::move(arg);
    return Init_SportModeState_foot_force(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_yaw_speed
{
public:
  explicit Init_SportModeState_yaw_speed(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_range_obstacle yaw_speed(::unitree_go::msg::SportModeState::_yaw_speed_type arg)
  {
    msg_.yaw_speed = std::move(arg);
    return Init_SportModeState_range_obstacle(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_velocity
{
public:
  explicit Init_SportModeState_velocity(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_yaw_speed velocity(::unitree_go::msg::SportModeState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_SportModeState_yaw_speed(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_body_height
{
public:
  explicit Init_SportModeState_body_height(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_velocity body_height(::unitree_go::msg::SportModeState::_body_height_type arg)
  {
    msg_.body_height = std::move(arg);
    return Init_SportModeState_velocity(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_position
{
public:
  explicit Init_SportModeState_position(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_body_height position(::unitree_go::msg::SportModeState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_SportModeState_body_height(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_foot_raise_height
{
public:
  explicit Init_SportModeState_foot_raise_height(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_position foot_raise_height(::unitree_go::msg::SportModeState::_foot_raise_height_type arg)
  {
    msg_.foot_raise_height = std::move(arg);
    return Init_SportModeState_position(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_gait_type
{
public:
  explicit Init_SportModeState_gait_type(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_foot_raise_height gait_type(::unitree_go::msg::SportModeState::_gait_type_type arg)
  {
    msg_.gait_type = std::move(arg);
    return Init_SportModeState_foot_raise_height(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_progress
{
public:
  explicit Init_SportModeState_progress(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_gait_type progress(::unitree_go::msg::SportModeState::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_SportModeState_gait_type(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_mode
{
public:
  explicit Init_SportModeState_mode(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_progress mode(::unitree_go::msg::SportModeState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_SportModeState_progress(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_imu_state
{
public:
  explicit Init_SportModeState_imu_state(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_mode imu_state(::unitree_go::msg::SportModeState::_imu_state_type arg)
  {
    msg_.imu_state = std::move(arg);
    return Init_SportModeState_mode(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_error_code
{
public:
  explicit Init_SportModeState_error_code(::unitree_go::msg::SportModeState & msg)
  : msg_(msg)
  {}
  Init_SportModeState_imu_state error_code(::unitree_go::msg::SportModeState::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_SportModeState_imu_state(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

class Init_SportModeState_stamp
{
public:
  Init_SportModeState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SportModeState_error_code stamp(::unitree_go::msg::SportModeState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_SportModeState_error_code(msg_);
  }

private:
  ::unitree_go::msg::SportModeState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::SportModeState>()
{
  return unitree_go::msg::builder::Init_SportModeState_stamp();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__BUILDER_HPP_
