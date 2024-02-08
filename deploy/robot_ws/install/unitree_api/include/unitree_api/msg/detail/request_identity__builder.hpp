// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/RequestIdentity.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__BUILDER_HPP_

#include "unitree_api/msg/detail/request_identity__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_RequestIdentity_api_id
{
public:
  explicit Init_RequestIdentity_api_id(::unitree_api::msg::RequestIdentity & msg)
  : msg_(msg)
  {}
  ::unitree_api::msg::RequestIdentity api_id(::unitree_api::msg::RequestIdentity::_api_id_type arg)
  {
    msg_.api_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::RequestIdentity msg_;
};

class Init_RequestIdentity_id
{
public:
  Init_RequestIdentity_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestIdentity_api_id id(::unitree_api::msg::RequestIdentity::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_RequestIdentity_api_id(msg_);
  }

private:
  ::unitree_api::msg::RequestIdentity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::RequestIdentity>()
{
  return unitree_api::msg::builder::Init_RequestIdentity_id();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__BUILDER_HPP_
