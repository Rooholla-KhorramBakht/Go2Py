// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/ResponseHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__BUILDER_HPP_

#include "unitree_api/msg/detail/response_header__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_ResponseHeader_status
{
public:
  explicit Init_ResponseHeader_status(::unitree_api::msg::ResponseHeader & msg)
  : msg_(msg)
  {}
  ::unitree_api::msg::ResponseHeader status(::unitree_api::msg::ResponseHeader::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::ResponseHeader msg_;
};

class Init_ResponseHeader_identity
{
public:
  Init_ResponseHeader_identity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ResponseHeader_status identity(::unitree_api::msg::ResponseHeader::_identity_type arg)
  {
    msg_.identity = std::move(arg);
    return Init_ResponseHeader_status(msg_);
  }

private:
  ::unitree_api::msg::ResponseHeader msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::ResponseHeader>()
{
  return unitree_api::msg::builder::Init_ResponseHeader_identity();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__BUILDER_HPP_
