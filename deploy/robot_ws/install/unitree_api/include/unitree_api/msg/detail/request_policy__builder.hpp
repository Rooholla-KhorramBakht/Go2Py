// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/RequestPolicy.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_POLICY__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_POLICY__BUILDER_HPP_

#include "unitree_api/msg/detail/request_policy__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_RequestPolicy_noreply
{
public:
  explicit Init_RequestPolicy_noreply(::unitree_api::msg::RequestPolicy & msg)
  : msg_(msg)
  {}
  ::unitree_api::msg::RequestPolicy noreply(::unitree_api::msg::RequestPolicy::_noreply_type arg)
  {
    msg_.noreply = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::RequestPolicy msg_;
};

class Init_RequestPolicy_priority
{
public:
  Init_RequestPolicy_priority()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestPolicy_noreply priority(::unitree_api::msg::RequestPolicy::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_RequestPolicy_noreply(msg_);
  }

private:
  ::unitree_api::msg::RequestPolicy msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::RequestPolicy>()
{
  return unitree_api::msg::builder::Init_RequestPolicy_priority();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_POLICY__BUILDER_HPP_
