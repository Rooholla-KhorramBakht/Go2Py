// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/PathPoint.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__PATH_POINT__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__PATH_POINT__BUILDER_HPP_

#include "unitree_go/msg/detail/path_point__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_PathPoint_vyaw
{
public:
  explicit Init_PathPoint_vyaw(::unitree_go::msg::PathPoint & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::PathPoint vyaw(::unitree_go::msg::PathPoint::_vyaw_type arg)
  {
    msg_.vyaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

class Init_PathPoint_vy
{
public:
  explicit Init_PathPoint_vy(::unitree_go::msg::PathPoint & msg)
  : msg_(msg)
  {}
  Init_PathPoint_vyaw vy(::unitree_go::msg::PathPoint::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_PathPoint_vyaw(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

class Init_PathPoint_vx
{
public:
  explicit Init_PathPoint_vx(::unitree_go::msg::PathPoint & msg)
  : msg_(msg)
  {}
  Init_PathPoint_vy vx(::unitree_go::msg::PathPoint::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_PathPoint_vy(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

class Init_PathPoint_yaw
{
public:
  explicit Init_PathPoint_yaw(::unitree_go::msg::PathPoint & msg)
  : msg_(msg)
  {}
  Init_PathPoint_vx yaw(::unitree_go::msg::PathPoint::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PathPoint_vx(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

class Init_PathPoint_y
{
public:
  explicit Init_PathPoint_y(::unitree_go::msg::PathPoint & msg)
  : msg_(msg)
  {}
  Init_PathPoint_yaw y(::unitree_go::msg::PathPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PathPoint_yaw(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

class Init_PathPoint_x
{
public:
  explicit Init_PathPoint_x(::unitree_go::msg::PathPoint & msg)
  : msg_(msg)
  {}
  Init_PathPoint_y x(::unitree_go::msg::PathPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PathPoint_y(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

class Init_PathPoint_t_from_start
{
public:
  Init_PathPoint_t_from_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathPoint_x t_from_start(::unitree_go::msg::PathPoint::_t_from_start_type arg)
  {
    msg_.t_from_start = std::move(arg);
    return Init_PathPoint_x(msg_);
  }

private:
  ::unitree_go::msg::PathPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::PathPoint>()
{
  return unitree_go::msg::builder::Init_PathPoint_t_from_start();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__PATH_POINT__BUILDER_HPP_
