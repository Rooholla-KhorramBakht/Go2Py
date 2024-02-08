// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_api:msg/ResponseStatus.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_STATUS__BUILDER_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_STATUS__BUILDER_HPP_

#include "unitree_api/msg/detail/response_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_api
{

namespace msg
{

namespace builder
{

class Init_ResponseStatus_code
{
public:
  Init_ResponseStatus_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::unitree_api::msg::ResponseStatus code(::unitree_api::msg::ResponseStatus::_code_type arg)
  {
    msg_.code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_api::msg::ResponseStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_api::msg::ResponseStatus>()
{
  return unitree_api::msg::builder::Init_ResponseStatus_code();
}

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_STATUS__BUILDER_HPP_
