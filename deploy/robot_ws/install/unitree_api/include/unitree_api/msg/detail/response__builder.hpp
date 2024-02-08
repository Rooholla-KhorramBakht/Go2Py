// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE__BUILDER_HPP_

#include "unitree_api/msg/detail/response__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_Response_binary
{
public:
  explicit Init_Response_binary(::unitree_api::msg::Response & msg)
  : msg_(msg)
  {}
  ::unitree_api::msg::Response binary(::unitree_api::msg::Response::_binary_type arg)
  {
    msg_.binary = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::Response msg_;
};

class Init_Response_data
{
public:
  explicit Init_Response_data(::unitree_api::msg::Response & msg)
  : msg_(msg)
  {}
  Init_Response_binary data(::unitree_api::msg::Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_Response_binary(msg_);
  }

private:
  ::unitree_api::msg::Response msg_;
};

class Init_Response_header
{
public:
  Init_Response_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Response_data header(::unitree_api::msg::Response::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Response_data(msg_);
  }

private:
  ::unitree_api::msg::Response msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::Response>()
{
  return unitree_api::msg::builder::Init_Response_header();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE__BUILDER_HPP_
