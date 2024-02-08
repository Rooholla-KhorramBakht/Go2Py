// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__ERROR__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__ERROR__BUILDER_HPP_

#include "unitree_go/msg/detail/error__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_Error_state
{
public:
  explicit Init_Error_state(::unitree_go::msg::Error & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::Error state(::unitree_go::msg::Error::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::Error msg_;
};

class Init_Error_source
{
public:
  Init_Error_source()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Error_state source(::unitree_go::msg::Error::_source_type arg)
  {
    msg_.source = std::move(arg);
    return Init_Error_state(msg_);
  }

private:
  ::unitree_go::msg::Error msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::Error>()
{
  return unitree_go::msg::builder::Init_Error_source();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__ERROR__BUILDER_HPP_
