// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/RequestLease.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_LEASE__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_LEASE__BUILDER_HPP_

#include "unitree_api/msg/detail/request_lease__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_RequestLease_id
{
public:
  Init_RequestLease_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::unitree_api::msg::RequestLease id(::unitree_api::msg::RequestLease::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::RequestLease msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::RequestLease>()
{
  return unitree_api::msg::builder::Init_RequestLease_id();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_LEASE__BUILDER_HPP_
