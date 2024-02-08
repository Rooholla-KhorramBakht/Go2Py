// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/Request.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST__BUILDER_HPP_

#include "unitree_api/msg/detail/request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_Request_binary
{
public:
  explicit Init_Request_binary(::unitree_api::msg::Request & msg)
  : msg_(msg)
  {}
  ::unitree_api::msg::Request binary(::unitree_api::msg::Request::_binary_type arg)
  {
    msg_.binary = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::Request msg_;
};

class Init_Request_parameter
{
public:
  explicit Init_Request_parameter(::unitree_api::msg::Request & msg)
  : msg_(msg)
  {}
  Init_Request_binary parameter(::unitree_api::msg::Request::_parameter_type arg)
  {
    msg_.parameter = std::move(arg);
    return Init_Request_binary(msg_);
  }

private:
  ::unitree_api::msg::Request msg_;
};

class Init_Request_header
{
public:
  Init_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Request_parameter header(::unitree_api::msg::Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Request_parameter(msg_);
  }

private:
  ::unitree_api::msg::Request msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::Request>()
{
  return unitree_api::msg::builder::Init_Request_header();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST__BUILDER_HPP_
