// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/Req.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__REQ__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__REQ__BUILDER_HPP_

#include "unitree_go/msg/detail/req__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_Req_body
{
public:
  explicit Init_Req_body(::unitree_go::msg::Req & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::Req body(::unitree_go::msg::Req::_body_type arg)
  {
    msg_.body = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::Req msg_;
};

class Init_Req_uuid
{
public:
  Init_Req_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Req_body uuid(::unitree_go::msg::Req::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_Req_body(msg_);
  }

private:
  ::unitree_go::msg::Req msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::Req>()
{
  return unitree_go::msg::builder::Init_Req_uuid();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__REQ__BUILDER_HPP_
