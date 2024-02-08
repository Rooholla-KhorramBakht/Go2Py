// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_HEADER__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_HEADER__BUILDER_HPP_

#include "unitree_api/msg/detail/request_header__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_RequestHeader_policy
{
public:
  explicit Init_RequestHeader_policy(::unitree_api::msg::RequestHeader & msg)
  : msg_(msg)
  {}
  ::unitree_api::msg::RequestHeader policy(::unitree_api::msg::RequestHeader::_policy_type arg)
  {
    msg_.policy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::RequestHeader msg_;
};

class Init_RequestHeader_lease
{
public:
  explicit Init_RequestHeader_lease(::unitree_api::msg::RequestHeader & msg)
  : msg_(msg)
  {}
  Init_RequestHeader_policy lease(::unitree_api::msg::RequestHeader::_lease_type arg)
  {
    msg_.lease = std::move(arg);
    return Init_RequestHeader_policy(msg_);
  }

private:
  ::unitree_api::msg::RequestHeader msg_;
};

class Init_RequestHeader_identity
{
public:
  Init_RequestHeader_identity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestHeader_lease identity(::unitree_api::msg::RequestHeader::_identity_type arg)
  {
    msg_.identity = std::move(arg);
    return Init_RequestHeader_lease(msg_);
  }

private:
  ::unitree_api::msg::RequestHeader msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::RequestHeader>()
{
  return unitree_api::msg::builder::Init_RequestHeader_identity();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_HEADER__BUILDER_HPP_
