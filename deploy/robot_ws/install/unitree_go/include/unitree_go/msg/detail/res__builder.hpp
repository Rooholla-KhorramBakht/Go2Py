// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/Res.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__RES__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__RES__BUILDER_HPP_

#include "unitree_go/msg/detail/res__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_Res_body
{
public:
  explicit Init_Res_body(::unitree_go::msg::Res & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::Res body(::unitree_go::msg::Res::_body_type arg)
  {
    msg_.body = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::Res msg_;
};

class Init_Res_data
{
public:
  explicit Init_Res_data(::unitree_go::msg::Res & msg)
  : msg_(msg)
  {}
  Init_Res_body data(::unitree_go::msg::Res::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_Res_body(msg_);
  }

private:
  ::unitree_go::msg::Res msg_;
};

class Init_Res_uuid
{
public:
  Init_Res_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Res_data uuid(::unitree_go::msg::Res::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_Res_data(msg_);
  }

private:
  ::unitree_go::msg::Res msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::Res>()
{
  return unitree_go::msg::builder::Init_Res_uuid();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__RES__BUILDER_HPP_
