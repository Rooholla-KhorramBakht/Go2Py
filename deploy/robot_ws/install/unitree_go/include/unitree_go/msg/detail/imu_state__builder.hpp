// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/IMUState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__IMU_STATE__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__IMU_STATE__BUILDER_HPP_

#include "unitree_go/msg/detail/imu_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_IMUState_temperature
{
public:
  explicit Init_IMUState_temperature(::unitree_go::msg::IMUState & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::IMUState temperature(::unitree_go::msg::IMUState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::IMUState msg_;
};

class Init_IMUState_rpy
{
public:
  explicit Init_IMUState_rpy(::unitree_go::msg::IMUState & msg)
  : msg_(msg)
  {}
  Init_IMUState_temperature rpy(::unitree_go::msg::IMUState::_rpy_type arg)
  {
    msg_.rpy = std::move(arg);
    return Init_IMUState_temperature(msg_);
  }

private:
  ::unitree_go::msg::IMUState msg_;
};

class Init_IMUState_accelerometer
{
public:
  explicit Init_IMUState_accelerometer(::unitree_go::msg::IMUState & msg)
  : msg_(msg)
  {}
  Init_IMUState_rpy accelerometer(::unitree_go::msg::IMUState::_accelerometer_type arg)
  {
    msg_.accelerometer = std::move(arg);
    return Init_IMUState_rpy(msg_);
  }

private:
  ::unitree_go::msg::IMUState msg_;
};

class Init_IMUState_gyroscope
{
public:
  explicit Init_IMUState_gyroscope(::unitree_go::msg::IMUState & msg)
  : msg_(msg)
  {}
  Init_IMUState_accelerometer gyroscope(::unitree_go::msg::IMUState::_gyroscope_type arg)
  {
    msg_.gyroscope = std::move(arg);
    return Init_IMUState_accelerometer(msg_);
  }

private:
  ::unitree_go::msg::IMUState msg_;
};

class Init_IMUState_quaternion
{
public:
  Init_IMUState_quaternion()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMUState_gyroscope quaternion(::unitree_go::msg::IMUState::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_IMUState_gyroscope(msg_);
  }

private:
  ::unitree_go::msg::IMUState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::IMUState>()
{
  return unitree_go::msg::builder::Init_IMUState_quaternion();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__IMU_STATE__BUILDER_HPP_
