// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/LidarState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LIDAR_STATE__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__LIDAR_STATE__BUILDER_HPP_

#include "unitree_go/msg/detail/lidar_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_LidarState_serial_buffer_read
{
public:
  explicit Init_LidarState_serial_buffer_read(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  ::unitree_go::msg::LidarState serial_buffer_read(::unitree_go::msg::LidarState::_serial_buffer_read_type arg)
  {
    msg_.serial_buffer_read = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_serial_buffer_size
{
public:
  explicit Init_LidarState_serial_buffer_size(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_serial_buffer_read serial_buffer_size(::unitree_go::msg::LidarState::_serial_buffer_size_type arg)
  {
    msg_.serial_buffer_size = std::move(arg);
    return Init_LidarState_serial_buffer_read(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_serial_recv_stamp
{
public:
  explicit Init_LidarState_serial_recv_stamp(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_serial_buffer_size serial_recv_stamp(::unitree_go::msg::LidarState::_serial_recv_stamp_type arg)
  {
    msg_.serial_recv_stamp = std::move(arg);
    return Init_LidarState_serial_buffer_size(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_imu_rpy
{
public:
  explicit Init_LidarState_imu_rpy(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_serial_recv_stamp imu_rpy(::unitree_go::msg::LidarState::_imu_rpy_type arg)
  {
    msg_.imu_rpy = std::move(arg);
    return Init_LidarState_serial_recv_stamp(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_imu_packet_loss_rate
{
public:
  explicit Init_LidarState_imu_packet_loss_rate(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_imu_rpy imu_packet_loss_rate(::unitree_go::msg::LidarState::_imu_packet_loss_rate_type arg)
  {
    msg_.imu_packet_loss_rate = std::move(arg);
    return Init_LidarState_imu_rpy(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_imu_frequency
{
public:
  explicit Init_LidarState_imu_frequency(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_imu_packet_loss_rate imu_frequency(::unitree_go::msg::LidarState::_imu_frequency_type arg)
  {
    msg_.imu_frequency = std::move(arg);
    return Init_LidarState_imu_packet_loss_rate(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_cloud_scan_num
{
public:
  explicit Init_LidarState_cloud_scan_num(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_imu_frequency cloud_scan_num(::unitree_go::msg::LidarState::_cloud_scan_num_type arg)
  {
    msg_.cloud_scan_num = std::move(arg);
    return Init_LidarState_imu_frequency(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_cloud_size
{
public:
  explicit Init_LidarState_cloud_size(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_cloud_scan_num cloud_size(::unitree_go::msg::LidarState::_cloud_size_type arg)
  {
    msg_.cloud_size = std::move(arg);
    return Init_LidarState_cloud_scan_num(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_cloud_packet_loss_rate
{
public:
  explicit Init_LidarState_cloud_packet_loss_rate(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_cloud_size cloud_packet_loss_rate(::unitree_go::msg::LidarState::_cloud_packet_loss_rate_type arg)
  {
    msg_.cloud_packet_loss_rate = std::move(arg);
    return Init_LidarState_cloud_size(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_cloud_frequency
{
public:
  explicit Init_LidarState_cloud_frequency(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_cloud_packet_loss_rate cloud_frequency(::unitree_go::msg::LidarState::_cloud_frequency_type arg)
  {
    msg_.cloud_frequency = std::move(arg);
    return Init_LidarState_cloud_packet_loss_rate(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_error_state
{
public:
  explicit Init_LidarState_error_state(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_cloud_frequency error_state(::unitree_go::msg::LidarState::_error_state_type arg)
  {
    msg_.error_state = std::move(arg);
    return Init_LidarState_cloud_frequency(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_com_rotation_speed
{
public:
  explicit Init_LidarState_com_rotation_speed(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_error_state com_rotation_speed(::unitree_go::msg::LidarState::_com_rotation_speed_type arg)
  {
    msg_.com_rotation_speed = std::move(arg);
    return Init_LidarState_error_state(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_sys_rotation_speed
{
public:
  explicit Init_LidarState_sys_rotation_speed(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_com_rotation_speed sys_rotation_speed(::unitree_go::msg::LidarState::_sys_rotation_speed_type arg)
  {
    msg_.sys_rotation_speed = std::move(arg);
    return Init_LidarState_com_rotation_speed(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_sdk_version
{
public:
  explicit Init_LidarState_sdk_version(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_sys_rotation_speed sdk_version(::unitree_go::msg::LidarState::_sdk_version_type arg)
  {
    msg_.sdk_version = std::move(arg);
    return Init_LidarState_sys_rotation_speed(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_software_version
{
public:
  explicit Init_LidarState_software_version(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_sdk_version software_version(::unitree_go::msg::LidarState::_software_version_type arg)
  {
    msg_.software_version = std::move(arg);
    return Init_LidarState_sdk_version(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_firmware_version
{
public:
  explicit Init_LidarState_firmware_version(::unitree_go::msg::LidarState & msg)
  : msg_(msg)
  {}
  Init_LidarState_software_version firmware_version(::unitree_go::msg::LidarState::_firmware_version_type arg)
  {
    msg_.firmware_version = std::move(arg);
    return Init_LidarState_software_version(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

class Init_LidarState_stamp
{
public:
  Init_LidarState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarState_firmware_version stamp(::unitree_go::msg::LidarState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_LidarState_firmware_version(msg_);
  }

private:
  ::unitree_go::msg::LidarState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::LidarState>()
{
  return unitree_go::msg::builder::Init_LidarState_stamp();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__LIDAR_STATE__BUILDER_HPP_
