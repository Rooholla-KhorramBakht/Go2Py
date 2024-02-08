// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/UwbState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__UWB_STATE__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__UWB_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__UwbState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__UwbState __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UwbState_
{
  using Type = UwbState_<ContainerAllocator>;

  explicit UwbState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->version.begin(), this->version.end(), 0);
      this->channel = 0;
      this->joy_mode = 0;
      this->orientation_est = 0.0f;
      this->pitch_est = 0.0f;
      this->distance_est = 0.0f;
      this->yaw_est = 0.0f;
      this->tag_roll = 0.0f;
      this->tag_pitch = 0.0f;
      this->tag_yaw = 0.0f;
      this->base_roll = 0.0f;
      this->base_pitch = 0.0f;
      this->base_yaw = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->joystick.begin(), this->joystick.end(), 0.0f);
      this->error_state = 0;
      this->buttons = 0;
      this->enabled_from_app = 0;
    }
  }

  explicit UwbState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : version(_alloc),
    joystick(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 2>::iterator, uint8_t>(this->version.begin(), this->version.end(), 0);
      this->channel = 0;
      this->joy_mode = 0;
      this->orientation_est = 0.0f;
      this->pitch_est = 0.0f;
      this->distance_est = 0.0f;
      this->yaw_est = 0.0f;
      this->tag_roll = 0.0f;
      this->tag_pitch = 0.0f;
      this->tag_yaw = 0.0f;
      this->base_roll = 0.0f;
      this->base_pitch = 0.0f;
      this->base_yaw = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->joystick.begin(), this->joystick.end(), 0.0f);
      this->error_state = 0;
      this->buttons = 0;
      this->enabled_from_app = 0;
    }
  }

  // field types and members
  using _version_type =
    std::array<uint8_t, 2>;
  _version_type version;
  using _channel_type =
    uint8_t;
  _channel_type channel;
  using _joy_mode_type =
    uint8_t;
  _joy_mode_type joy_mode;
  using _orientation_est_type =
    float;
  _orientation_est_type orientation_est;
  using _pitch_est_type =
    float;
  _pitch_est_type pitch_est;
  using _distance_est_type =
    float;
  _distance_est_type distance_est;
  using _yaw_est_type =
    float;
  _yaw_est_type yaw_est;
  using _tag_roll_type =
    float;
  _tag_roll_type tag_roll;
  using _tag_pitch_type =
    float;
  _tag_pitch_type tag_pitch;
  using _tag_yaw_type =
    float;
  _tag_yaw_type tag_yaw;
  using _base_roll_type =
    float;
  _base_roll_type base_roll;
  using _base_pitch_type =
    float;
  _base_pitch_type base_pitch;
  using _base_yaw_type =
    float;
  _base_yaw_type base_yaw;
  using _joystick_type =
    std::array<float, 2>;
  _joystick_type joystick;
  using _error_state_type =
    uint8_t;
  _error_state_type error_state;
  using _buttons_type =
    uint8_t;
  _buttons_type buttons;
  using _enabled_from_app_type =
    uint8_t;
  _enabled_from_app_type enabled_from_app;

  // setters for named parameter idiom
  Type & set__version(
    const std::array<uint8_t, 2> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__channel(
    const uint8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__joy_mode(
    const uint8_t & _arg)
  {
    this->joy_mode = _arg;
    return *this;
  }
  Type & set__orientation_est(
    const float & _arg)
  {
    this->orientation_est = _arg;
    return *this;
  }
  Type & set__pitch_est(
    const float & _arg)
  {
    this->pitch_est = _arg;
    return *this;
  }
  Type & set__distance_est(
    const float & _arg)
  {
    this->distance_est = _arg;
    return *this;
  }
  Type & set__yaw_est(
    const float & _arg)
  {
    this->yaw_est = _arg;
    return *this;
  }
  Type & set__tag_roll(
    const float & _arg)
  {
    this->tag_roll = _arg;
    return *this;
  }
  Type & set__tag_pitch(
    const float & _arg)
  {
    this->tag_pitch = _arg;
    return *this;
  }
  Type & set__tag_yaw(
    const float & _arg)
  {
    this->tag_yaw = _arg;
    return *this;
  }
  Type & set__base_roll(
    const float & _arg)
  {
    this->base_roll = _arg;
    return *this;
  }
  Type & set__base_pitch(
    const float & _arg)
  {
    this->base_pitch = _arg;
    return *this;
  }
  Type & set__base_yaw(
    const float & _arg)
  {
    this->base_yaw = _arg;
    return *this;
  }
  Type & set__joystick(
    const std::array<float, 2> & _arg)
  {
    this->joystick = _arg;
    return *this;
  }
  Type & set__error_state(
    const uint8_t & _arg)
  {
    this->error_state = _arg;
    return *this;
  }
  Type & set__buttons(
    const uint8_t & _arg)
  {
    this->buttons = _arg;
    return *this;
  }
  Type & set__enabled_from_app(
    const uint8_t & _arg)
  {
    this->enabled_from_app = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::UwbState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::UwbState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::UwbState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::UwbState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::UwbState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::UwbState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::UwbState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::UwbState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::UwbState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::UwbState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__UwbState
    std::shared_ptr<unitree_go::msg::UwbState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__UwbState
    std::shared_ptr<unitree_go::msg::UwbState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UwbState_ & other) const
  {
    if (this->version != other.version) {
      return false;
    }
    if (this->channel != other.channel) {
      return false;
    }
    if (this->joy_mode != other.joy_mode) {
      return false;
    }
    if (this->orientation_est != other.orientation_est) {
      return false;
    }
    if (this->pitch_est != other.pitch_est) {
      return false;
    }
    if (this->distance_est != other.distance_est) {
      return false;
    }
    if (this->yaw_est != other.yaw_est) {
      return false;
    }
    if (this->tag_roll != other.tag_roll) {
      return false;
    }
    if (this->tag_pitch != other.tag_pitch) {
      return false;
    }
    if (this->tag_yaw != other.tag_yaw) {
      return false;
    }
    if (this->base_roll != other.base_roll) {
      return false;
    }
    if (this->base_pitch != other.base_pitch) {
      return false;
    }
    if (this->base_yaw != other.base_yaw) {
      return false;
    }
    if (this->joystick != other.joystick) {
      return false;
    }
    if (this->error_state != other.error_state) {
      return false;
    }
    if (this->buttons != other.buttons) {
      return false;
    }
    if (this->enabled_from_app != other.enabled_from_app) {
      return false;
    }
    return true;
  }
  bool operator!=(const UwbState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UwbState_

// alias to use template instance with default allocator
using UwbState =
  unitree_go::msg::UwbState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__UWB_STATE__STRUCT_HPP_
