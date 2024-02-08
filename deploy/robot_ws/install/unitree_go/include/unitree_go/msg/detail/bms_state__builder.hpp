// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__BMS_STATE__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__BMS_STATE__BUILDER_HPP_

#include "unitree_go/msg/detail/bms_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_BmsState_cell_vol
{
public:
  explicit Init_BmsState_cell_vol(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::BmsState cell_vol(::unitree_go::msg::BmsState::_cell_vol_type arg)
  {
    msg_.cell_vol = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_mcu_ntc
{
public:
  explicit Init_BmsState_mcu_ntc(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_cell_vol mcu_ntc(::unitree_go::msg::BmsState::_mcu_ntc_type arg)
  {
    msg_.mcu_ntc = std::move(arg);
    return Init_BmsState_cell_vol(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_bq_ntc
{
public:
  explicit Init_BmsState_bq_ntc(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_mcu_ntc bq_ntc(::unitree_go::msg::BmsState::_bq_ntc_type arg)
  {
    msg_.bq_ntc = std::move(arg);
    return Init_BmsState_mcu_ntc(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_cycle
{
public:
  explicit Init_BmsState_cycle(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_bq_ntc cycle(::unitree_go::msg::BmsState::_cycle_type arg)
  {
    msg_.cycle = std::move(arg);
    return Init_BmsState_bq_ntc(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_current
{
public:
  explicit Init_BmsState_current(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_cycle current(::unitree_go::msg::BmsState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BmsState_cycle(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_soc
{
public:
  explicit Init_BmsState_soc(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_current soc(::unitree_go::msg::BmsState::_soc_type arg)
  {
    msg_.soc = std::move(arg);
    return Init_BmsState_current(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_status
{
public:
  explicit Init_BmsState_status(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_soc status(::unitree_go::msg::BmsState::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_BmsState_soc(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_version_low
{
public:
  explicit Init_BmsState_version_low(::unitree_go::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_status version_low(::unitree_go::msg::BmsState::_version_low_type arg)
  {
    msg_.version_low = std::move(arg);
    return Init_BmsState_status(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

class Init_BmsState_version_high
{
public:
  Init_BmsState_version_high()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BmsState_version_low version_high(::unitree_go::msg::BmsState::_version_high_type arg)
  {
    msg_.version_high = std::move(arg);
    return Init_BmsState_version_low(msg_);
  }

private:
  ::unitree_go::msg::BmsState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::BmsState>()
{
  return unitree_go::msg::builder::Init_BmsState_version_high();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__BMS_STATE__BUILDER_HPP_
