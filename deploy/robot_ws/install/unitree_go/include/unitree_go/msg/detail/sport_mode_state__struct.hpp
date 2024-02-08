// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/SportModeState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'stamp'
#include "unitree_go/msg/detail/time_spec__struct.hpp"
// Member 'imu_state'
#include "unitree_go/msg/detail/imu_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__SportModeState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__SportModeState __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SportModeState_
{
  using Type = SportModeState_<ContainerAllocator>;

  explicit SportModeState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    imu_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_code = 0ul;
      this->mode = 0;
      this->progress = 0.0f;
      this->gait_type = 0;
      this->foot_raise_height = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->body_height = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->velocity.begin(), this->velocity.end(), 0.0f);
      this->yaw_speed = 0.0f;
      std::fill<typename std::array<float, 4>::iterator, float>(this->range_obstacle.begin(), this->range_obstacle.end(), 0.0f);
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->foot_force.begin(), this->foot_force.end(), 0);
      std::fill<typename std::array<float, 12>::iterator, float>(this->foot_position_body.begin(), this->foot_position_body.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->foot_speed_body.begin(), this->foot_speed_body.end(), 0.0f);
    }
  }

  explicit SportModeState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    imu_state(_alloc, _init),
    position(_alloc),
    velocity(_alloc),
    range_obstacle(_alloc),
    foot_force(_alloc),
    foot_position_body(_alloc),
    foot_speed_body(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_code = 0ul;
      this->mode = 0;
      this->progress = 0.0f;
      this->gait_type = 0;
      this->foot_raise_height = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->body_height = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->velocity.begin(), this->velocity.end(), 0.0f);
      this->yaw_speed = 0.0f;
      std::fill<typename std::array<float, 4>::iterator, float>(this->range_obstacle.begin(), this->range_obstacle.end(), 0.0f);
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->foot_force.begin(), this->foot_force.end(), 0);
      std::fill<typename std::array<float, 12>::iterator, float>(this->foot_position_body.begin(), this->foot_position_body.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->foot_speed_body.begin(), this->foot_speed_body.end(), 0.0f);
    }
  }

  // field types and members
  using _stamp_type =
    unitree_go::msg::TimeSpec_<ContainerAllocator>;
  _stamp_type stamp;
  using _error_code_type =
    uint32_t;
  _error_code_type error_code;
  using _imu_state_type =
    unitree_go::msg::IMUState_<ContainerAllocator>;
  _imu_state_type imu_state;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _progress_type =
    float;
  _progress_type progress;
  using _gait_type_type =
    uint8_t;
  _gait_type_type gait_type;
  using _foot_raise_height_type =
    float;
  _foot_raise_height_type foot_raise_height;
  using _position_type =
    std::array<float, 3>;
  _position_type position;
  using _body_height_type =
    float;
  _body_height_type body_height;
  using _velocity_type =
    std::array<float, 3>;
  _velocity_type velocity;
  using _yaw_speed_type =
    float;
  _yaw_speed_type yaw_speed;
  using _range_obstacle_type =
    std::array<float, 4>;
  _range_obstacle_type range_obstacle;
  using _foot_force_type =
    std::array<int16_t, 4>;
  _foot_force_type foot_force;
  using _foot_position_body_type =
    std::array<float, 12>;
  _foot_position_body_type foot_position_body;
  using _foot_speed_body_type =
    std::array<float, 12>;
  _foot_speed_body_type foot_speed_body;

  // setters for named parameter idiom
  Type & set__stamp(
    const unitree_go::msg::TimeSpec_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__imu_state(
    const unitree_go::msg::IMUState_<ContainerAllocator> & _arg)
  {
    this->imu_state = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__progress(
    const float & _arg)
  {
    this->progress = _arg;
    return *this;
  }
  Type & set__gait_type(
    const uint8_t & _arg)
  {
    this->gait_type = _arg;
    return *this;
  }
  Type & set__foot_raise_height(
    const float & _arg)
  {
    this->foot_raise_height = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<float, 3> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__body_height(
    const float & _arg)
  {
    this->body_height = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::array<float, 3> & _arg)
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
  Type & set__range_obstacle(
    const std::array<float, 4> & _arg)
  {
    this->range_obstacle = _arg;
    return *this;
  }
  Type & set__foot_force(
    const std::array<int16_t, 4> & _arg)
  {
    this->foot_force = _arg;
    return *this;
  }
  Type & set__foot_position_body(
    const std::array<float, 12> & _arg)
  {
    this->foot_position_body = _arg;
    return *this;
  }
  Type & set__foot_speed_body(
    const std::array<float, 12> & _arg)
  {
    this->foot_speed_body = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::SportModeState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::SportModeState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::SportModeState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::SportModeState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::SportModeState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::SportModeState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::SportModeState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::SportModeState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::SportModeState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::SportModeState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__SportModeState
    std::shared_ptr<unitree_go::msg::SportModeState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__SportModeState
    std::shared_ptr<unitree_go::msg::SportModeState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SportModeState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->imu_state != other.imu_state) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->progress != other.progress) {
      return false;
    }
    if (this->gait_type != other.gait_type) {
      return false;
    }
    if (this->foot_raise_height != other.foot_raise_height) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->body_height != other.body_height) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->yaw_speed != other.yaw_speed) {
      return false;
    }
    if (this->range_obstacle != other.range_obstacle) {
      return false;
    }
    if (this->foot_force != other.foot_force) {
      return false;
    }
    if (this->foot_position_body != other.foot_position_body) {
      return false;
    }
    if (this->foot_speed_body != other.foot_speed_body) {
      return false;
    }
    return true;
  }
  bool operator!=(const SportModeState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SportModeState_

// alias to use template instance with default allocator
using SportModeState =
  unitree_go::msg::SportModeState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__SPORT_MODE_STATE__STRUCT_HPP_
