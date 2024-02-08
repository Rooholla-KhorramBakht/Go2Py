// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/TimeSpec.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__TIME_SPEC__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__TIME_SPEC__BUILDER_HPP_

#include "unitree_go/msg/detail/time_spec__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_TimeSpec_nanosec
{
public:
  explicit Init_TimeSpec_nanosec(::unitree_go::msg::TimeSpec & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::TimeSpec nanosec(::unitree_go::msg::TimeSpec::_nanosec_type arg)
  {
    msg_.nanosec = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::TimeSpec msg_;
};

class Init_TimeSpec_sec
{
public:
  Init_TimeSpec_sec()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimeSpec_nanosec sec(::unitree_go::msg::TimeSpec::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return Init_TimeSpec_nanosec(msg_);
  }

private:
  ::unitree_go::msg::TimeSpec msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::TimeSpec>()
{
  return unitree_go::msg::builder::Init_TimeSpec_sec();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__TIME_SPEC__BUILDER_HPP_
