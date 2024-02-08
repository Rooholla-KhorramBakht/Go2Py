// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/LidarState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__LIDAR_STATE__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__LIDAR_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__LidarState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__LidarState __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarState_
{
  using Type = LidarState_<ContainerAllocator>;

  explicit LidarState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stamp = 0.0;
      this->firmware_version = "";
      this->software_version = "";
      this->sdk_version = "";
      this->sys_rotation_speed = 0.0f;
      this->com_rotation_speed = 0.0f;
      this->error_state = 0;
      this->cloud_frequency = 0.0f;
      this->cloud_packet_loss_rate = 0.0f;
      this->cloud_size = 0ul;
      this->cloud_scan_num = 0ul;
      this->imu_frequency = 0.0f;
      this->imu_packet_loss_rate = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->imu_rpy.begin(), this->imu_rpy.end(), 0.0f);
      this->serial_recv_stamp = 0.0;
      this->serial_buffer_size = 0ul;
      this->serial_buffer_read = 0ul;
    }
  }

  explicit LidarState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : firmware_version(_alloc),
    software_version(_alloc),
    sdk_version(_alloc),
    imu_rpy(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stamp = 0.0;
      this->firmware_version = "";
      this->software_version = "";
      this->sdk_version = "";
      this->sys_rotation_speed = 0.0f;
      this->com_rotation_speed = 0.0f;
      this->error_state = 0;
      this->cloud_frequency = 0.0f;
      this->cloud_packet_loss_rate = 0.0f;
      this->cloud_size = 0ul;
      this->cloud_scan_num = 0ul;
      this->imu_frequency = 0.0f;
      this->imu_packet_loss_rate = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->imu_rpy.begin(), this->imu_rpy.end(), 0.0f);
      this->serial_recv_stamp = 0.0;
      this->serial_buffer_size = 0ul;
      this->serial_buffer_read = 0ul;
    }
  }

  // field types and members
  using _stamp_type =
    double;
  _stamp_type stamp;
  using _firmware_version_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _firmware_version_type firmware_version;
  using _software_version_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _software_version_type software_version;
  using _sdk_version_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _sdk_version_type sdk_version;
  using _sys_rotation_speed_type =
    float;
  _sys_rotation_speed_type sys_rotation_speed;
  using _com_rotation_speed_type =
    float;
  _com_rotation_speed_type com_rotation_speed;
  using _error_state_type =
    uint8_t;
  _error_state_type error_state;
  using _cloud_frequency_type =
    float;
  _cloud_frequency_type cloud_frequency;
  using _cloud_packet_loss_rate_type =
    float;
  _cloud_packet_loss_rate_type cloud_packet_loss_rate;
  using _cloud_size_type =
    uint32_t;
  _cloud_size_type cloud_size;
  using _cloud_scan_num_type =
    uint32_t;
  _cloud_scan_num_type cloud_scan_num;
  using _imu_frequency_type =
    float;
  _imu_frequency_type imu_frequency;
  using _imu_packet_loss_rate_type =
    float;
  _imu_packet_loss_rate_type imu_packet_loss_rate;
  using _imu_rpy_type =
    std::array<float, 3>;
  _imu_rpy_type imu_rpy;
  using _serial_recv_stamp_type =
    double;
  _serial_recv_stamp_type serial_recv_stamp;
  using _serial_buffer_size_type =
    uint32_t;
  _serial_buffer_size_type serial_buffer_size;
  using _serial_buffer_read_type =
    uint32_t;
  _serial_buffer_read_type serial_buffer_read;

  // setters for named parameter idiom
  Type & set__stamp(
    const double & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__firmware_version(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->firmware_version = _arg;
    return *this;
  }
  Type & set__software_version(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->software_version = _arg;
    return *this;
  }
  Type & set__sdk_version(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->sdk_version = _arg;
    return *this;
  }
  Type & set__sys_rotation_speed(
    const float & _arg)
  {
    this->sys_rotation_speed = _arg;
    return *this;
  }
  Type & set__com_rotation_speed(
    const float & _arg)
  {
    this->com_rotation_speed = _arg;
    return *this;
  }
  Type & set__error_state(
    const uint8_t & _arg)
  {
    this->error_state = _arg;
    return *this;
  }
  Type & set__cloud_frequency(
    const float & _arg)
  {
    this->cloud_frequency = _arg;
    return *this;
  }
  Type & set__cloud_packet_loss_rate(
    const float & _arg)
  {
    this->cloud_packet_loss_rate = _arg;
    return *this;
  }
  Type & set__cloud_size(
    const uint32_t & _arg)
  {
    this->cloud_size = _arg;
    return *this;
  }
  Type & set__cloud_scan_num(
    const uint32_t & _arg)
  {
    this->cloud_scan_num = _arg;
    return *this;
  }
  Type & set__imu_frequency(
    const float & _arg)
  {
    this->imu_frequency = _arg;
    return *this;
  }
  Type & set__imu_packet_loss_rate(
    const float & _arg)
  {
    this->imu_packet_loss_rate = _arg;
    return *this;
  }
  Type & set__imu_rpy(
    const std::array<float, 3> & _arg)
  {
    this->imu_rpy = _arg;
    return *this;
  }
  Type & set__serial_recv_stamp(
    const double & _arg)
  {
    this->serial_recv_stamp = _arg;
    return *this;
  }
  Type & set__serial_buffer_size(
    const uint32_t & _arg)
  {
    this->serial_buffer_size = _arg;
    return *this;
  }
  Type & set__serial_buffer_read(
    const uint32_t & _arg)
  {
    this->serial_buffer_read = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::LidarState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::LidarState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::LidarState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::LidarState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::LidarState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::LidarState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::LidarState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::LidarState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::LidarState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::LidarState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__LidarState
    std::shared_ptr<unitree_go::msg::LidarState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__LidarState
    std::shared_ptr<unitree_go::msg::LidarState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->firmware_version != other.firmware_version) {
      return false;
    }
    if (this->software_version != other.software_version) {
      return false;
    }
    if (this->sdk_version != other.sdk_version) {
      return false;
    }
    if (this->sys_rotation_speed != other.sys_rotation_speed) {
      return false;
    }
    if (this->com_rotation_speed != other.com_rotation_speed) {
      return false;
    }
    if (this->error_state != other.error_state) {
      return false;
    }
    if (this->cloud_frequency != other.cloud_frequency) {
      return false;
    }
    if (this->cloud_packet_loss_rate != other.cloud_packet_loss_rate) {
      return false;
    }
    if (this->cloud_size != other.cloud_size) {
      return false;
    }
    if (this->cloud_scan_num != other.cloud_scan_num) {
      return false;
    }
    if (this->imu_frequency != other.imu_frequency) {
      return false;
    }
    if (this->imu_packet_loss_rate != other.imu_packet_loss_rate) {
      return false;
    }
    if (this->imu_rpy != other.imu_rpy) {
      return false;
    }
    if (this->serial_recv_stamp != other.serial_recv_stamp) {
      return false;
    }
    if (this->serial_buffer_size != other.serial_buffer_size) {
      return false;
    }
    if (this->serial_buffer_read != other.serial_buffer_read) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarState_

// alias to use template instance with default allocator
using LidarState =
  unitree_go::msg::LidarState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__LIDAR_STATE__STRUCT_HPP_
