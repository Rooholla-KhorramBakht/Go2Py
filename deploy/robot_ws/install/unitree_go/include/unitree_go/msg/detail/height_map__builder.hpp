// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/HeightMap.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__BUILDER_HPP_

#include "unitree_go/msg/detail/height_map__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_HeightMap_data
{
public:
  explicit Init_HeightMap_data(::unitree_go::msg::HeightMap & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::HeightMap data(::unitree_go::msg::HeightMap::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

class Init_HeightMap_origin
{
public:
  explicit Init_HeightMap_origin(::unitree_go::msg::HeightMap & msg)
  : msg_(msg)
  {}
  Init_HeightMap_data origin(::unitree_go::msg::HeightMap::_origin_type arg)
  {
    msg_.origin = std::move(arg);
    return Init_HeightMap_data(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

class Init_HeightMap_height
{
public:
  explicit Init_HeightMap_height(::unitree_go::msg::HeightMap & msg)
  : msg_(msg)
  {}
  Init_HeightMap_origin height(::unitree_go::msg::HeightMap::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_HeightMap_origin(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

class Init_HeightMap_width
{
public:
  explicit Init_HeightMap_width(::unitree_go::msg::HeightMap & msg)
  : msg_(msg)
  {}
  Init_HeightMap_height width(::unitree_go::msg::HeightMap::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_HeightMap_height(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

class Init_HeightMap_resolution
{
public:
  explicit Init_HeightMap_resolution(::unitree_go::msg::HeightMap & msg)
  : msg_(msg)
  {}
  Init_HeightMap_width resolution(::unitree_go::msg::HeightMap::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return Init_HeightMap_width(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

class Init_HeightMap_frame_id
{
public:
  explicit Init_HeightMap_frame_id(::unitree_go::msg::HeightMap & msg)
  : msg_(msg)
  {}
  Init_HeightMap_resolution frame_id(::unitree_go::msg::HeightMap::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_HeightMap_resolution(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

class Init_HeightMap_stamp
{
public:
  Init_HeightMap_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeightMap_frame_id stamp(::unitree_go::msg::HeightMap::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_HeightMap_frame_id(msg_);
  }

private:
  ::unitree_go::msg::HeightMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::HeightMap>()
{
  return unitree_go::msg::builder::Init_HeightMap_stamp();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__HEIGHT_MAP__BUILDER_HPP_
