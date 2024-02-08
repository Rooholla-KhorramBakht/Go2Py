// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/AudioData.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__AUDIO_DATA__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__AUDIO_DATA__BUILDER_HPP_

#include "unitree_go/msg/detail/audio_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_AudioData_data
{
public:
  explicit Init_AudioData_data(::unitree_go::msg::AudioData & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::AudioData data(::unitree_go::msg::AudioData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::AudioData msg_;
};

class Init_AudioData_time_frame
{
public:
  Init_AudioData_time_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AudioData_data time_frame(::unitree_go::msg::AudioData::_time_frame_type arg)
  {
    msg_.time_frame = std::move(arg);
    return Init_AudioData_data(msg_);
  }

private:
  ::unitree_go::msg::AudioData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::AudioData>()
{
  return unitree_go::msg::builder::Init_AudioData_time_frame();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__AUDIO_DATA__BUILDER_HPP_
