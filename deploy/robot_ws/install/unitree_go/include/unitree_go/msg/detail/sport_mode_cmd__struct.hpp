// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'bms_cmd'
#include "unitree_go/msg/detail/bms_cmd__struct.hpp"
// Member 'path_point'
#include "unitree_go/msg/detail/path_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__SportModeCmd __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__SportModeCmd __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SportModeCmd_
{
  using Type = SportModeCmd_<ContainerAllocator>;

  explicit SportModeCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bms_cmd(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->gait_type = 0;
      this->speed_level = 0;
      this->foot_raise_height = 0.0f;
      this->body_height = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->euler.begin(), this->euler.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->velocity.begin(), this->velocity.end(), 0.0f);
      this->yaw_speed = 0.0f;
      this->path_point.fill(unitree_go::msg::PathPoint_<ContainerAllocator>{_init});
    }
  }

  explicit SportModeCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc),
    euler(_alloc),
    velocity(_alloc),
    bms_cmd(_alloc, _init),
    path_point(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->gait_type = 0;
      this->speed_level = 0;
      this->foot_raise_height = 0.0f;
      this->body_height = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->euler.begin(), this->euler.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->velocity.begin(), this->velocity.end(), 0.0f);
      this->yaw_speed = 0.0f;
      this->path_point.fill(unitree_go::msg::PathPoint_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _gait_type_type =
    uint8_t;
  _gait_type_type gait_type;
  using _speed_level_type =
    uint8_t;
  _speed_level_type speed_level;
  using _foot_raise_height_type =
    float;
  _foot_raise_height_type foot_raise_height;
  using _body_height_type =
    float;
  _body_height_type body_height;
  using _position_type =
    std::array<float, 2>;
  _position_type position;
  using _euler_type =
    std::array<float, 3>;
  _euler_type euler;
  using _velocity_type =
    std::array<float, 2>;
  _velocity_type velocity;
  using _yaw_speed_type =
    float;
  _yaw_speed_type yaw_speed;
  using _bms_cmd_type =
    unitree_go::msg::BmsCmd_<ContainerAllocator>;
  _bms_cmd_type bms_cmd;
  using _path_point_type =
    std::array<unitree_go::msg::PathPoint_<ContainerAllocator>, 30>;
  _path_point_type path_point;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__gait_type(
    const uint8_t & _arg)
  {
    this->gait_type = _arg;
    return *this;
  }
  Type & set__speed_level(
    const uint8_t & _arg)
  {
    this->speed_level = _arg;
    return *this;
  }
  Type & set__foot_raise_height(
    const float & _arg)
  {
    this->foot_raise_height = _arg;
    return *this;
  }
  Type & set__body_height(
    const float & _arg)
  {
    this->body_height = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<float, 2> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__euler(
    const std::array<float, 3> & _arg)
  {
    this->euler = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::array<float, 2> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__yaw_speed(
    const float & _arg)
  {
    this->yaw_speed = _arg;
    return *this;
  }
  Type & set__bms_cmd(
    const unitree_go::msg::BmsCmd_<ContainerAllocator> & _arg)
  {
    this->bms_cmd = _arg;
    return *this;
  }
  Type & set__path_point(
    const std::array<unitree_go::msg::PathPoint_<ContainerAllocator>, 30> & _arg)
  {
    this->path_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::SportModeCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::SportModeCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::SportModeCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::SportModeCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__SportModeCmd
    std::shared_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__SportModeCmd
    std::shared_ptr<unitree_go::msg::SportModeCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SportModeCmd_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->gait_type != other.gait_type) {
      return false;
    }
    if (this->speed_level != other.speed_level) {
      return false;
    }
    if (this->foot_raise_height != other.foot_raise_height) {
      return false;
    }
    if (this->body_height != other.body_height) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->euler != other.euler) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->yaw_speed != other.yaw_speed) {
      return false;
    }
    if (this->bms_cmd != other.bms_cmd) {
      return false;
    }
    if (this->path_point != other.path_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const SportModeCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SportModeCmd_

// alias to use template instance with default allocator
using SportModeCmd =
  unitree_go::msg::SportModeCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_CMD__STRUCT_HPP_
