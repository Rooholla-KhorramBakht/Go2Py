// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__LOW_STATE__BUILDER_HPP_

#include "unitree_go/msg/detail/low_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_LowState_crc
{
public:
  explicit Init_LowState_crc(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::LowState crc(::unitree_go::msg::LowState::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_reserve
{
public:
  explicit Init_LowState_reserve(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_crc reserve(::unitree_go::msg::LowState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_LowState_crc(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_fan_frequency
{
public:
  explicit Init_LowState_fan_frequency(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_reserve fan_frequency(::unitree_go::msg::LowState::_fan_frequency_type arg)
  {
    msg_.fan_frequency = std::move(arg);
    return Init_LowState_reserve(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_power_a
{
public:
  explicit Init_LowState_power_a(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_fan_frequency power_a(::unitree_go::msg::LowState::_power_a_type arg)
  {
    msg_.power_a = std::move(arg);
    return Init_LowState_fan_frequency(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_power_v
{
public:
  explicit Init_LowState_power_v(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_power_a power_v(::unitree_go::msg::LowState::_power_v_type arg)
  {
    msg_.power_v = std::move(arg);
    return Init_LowState_power_a(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_temperature_ntc2
{
public:
  explicit Init_LowState_temperature_ntc2(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_power_v temperature_ntc2(::unitree_go::msg::LowState::_temperature_ntc2_type arg)
  {
    msg_.temperature_ntc2 = std::move(arg);
    return Init_LowState_power_v(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_temperature_ntc1
{
public:
  explicit Init_LowState_temperature_ntc1(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_temperature_ntc2 temperature_ntc1(::unitree_go::msg::LowState::_temperature_ntc1_type arg)
  {
    msg_.temperature_ntc1 = std::move(arg);
    return Init_LowState_temperature_ntc2(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_adc_reel
{
public:
  explicit Init_LowState_adc_reel(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_temperature_ntc1 adc_reel(::unitree_go::msg::LowState::_adc_reel_type arg)
  {
    msg_.adc_reel = std::move(arg);
    return Init_LowState_temperature_ntc1(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_bit_flag
{
public:
  explicit Init_LowState_bit_flag(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_adc_reel bit_flag(::unitree_go::msg::LowState::_bit_flag_type arg)
  {
    msg_.bit_flag = std::move(arg);
    return Init_LowState_adc_reel(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_wireless_remote
{
public:
  explicit Init_LowState_wireless_remote(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_bit_flag wireless_remote(::unitree_go::msg::LowState::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_LowState_bit_flag(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_tick
{
public:
  explicit Init_LowState_tick(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_wireless_remote tick(::unitree_go::msg::LowState::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_LowState_wireless_remote(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_foot_force_est
{
public:
  explicit Init_LowState_foot_force_est(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_tick foot_force_est(::unitree_go::msg::LowState::_foot_force_est_type arg)
  {
    msg_.foot_force_est = std::move(arg);
    return Init_LowState_tick(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_foot_force
{
public:
  explicit Init_LowState_foot_force(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_foot_force_est foot_force(::unitree_go::msg::LowState::_foot_force_type arg)
  {
    msg_.foot_force = std::move(arg);
    return Init_LowState_foot_force_est(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_bms_state
{
public:
  explicit Init_LowState_bms_state(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_foot_force bms_state(::unitree_go::msg::LowState::_bms_state_type arg)
  {
    msg_.bms_state = std::move(arg);
    return Init_LowState_foot_force(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_motor_state
{
public:
  explicit Init_LowState_motor_state(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_bms_state motor_state(::unitree_go::msg::LowState::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return Init_LowState_bms_state(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_imu_state
{
public:
  explicit Init_LowState_imu_state(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_motor_state imu_state(::unitree_go::msg::LowState::_imu_state_type arg)
  {
    msg_.imu_state = std::move(arg);
    return Init_LowState_motor_state(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_bandwidth
{
public:
  explicit Init_LowState_bandwidth(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_imu_state bandwidth(::unitree_go::msg::LowState::_bandwidth_type arg)
  {
    msg_.bandwidth = std::move(arg);
    return Init_LowState_imu_state(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_version
{
public:
  explicit Init_LowState_version(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_bandwidth version(::unitree_go::msg::LowState::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_LowState_bandwidth(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_sn
{
public:
  explicit Init_LowState_sn(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_version sn(::unitree_go::msg::LowState::_sn_type arg)
  {
    msg_.sn = std::move(arg);
    return Init_LowState_version(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_frame_reserve
{
public:
  explicit Init_LowState_frame_reserve(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_sn frame_reserve(::unitree_go::msg::LowState::_frame_reserve_type arg)
  {
    msg_.frame_reserve = std::move(arg);
    return Init_LowState_sn(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_level_flag
{
public:
  explicit Init_LowState_level_flag(::unitree_go::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_frame_reserve level_flag(::unitree_go::msg::LowState::_level_flag_type arg)
  {
    msg_.level_flag = std::move(arg);
    return Init_LowState_frame_reserve(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

class Init_LowState_head
{
public:
  Init_LowState_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowState_level_flag head(::unitree_go::msg::LowState::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_LowState_level_flag(msg_);
  }

private:
  ::unitree_go::msg::LowState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::LowState>()
{
  return unitree_go::msg::builder::Init_LowState_head();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
