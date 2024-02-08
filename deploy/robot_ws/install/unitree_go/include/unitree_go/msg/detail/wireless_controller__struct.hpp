// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/WirelessController.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__WirelessController __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__WirelessController __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WirelessController_
{
  using Type = WirelessController_<ContainerAllocator>;

  explicit WirelessController_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lx = 0.0f;
      this->ly = 0.0f;
      this->rx = 0.0f;
      this->ry = 0.0f;
      this->keys = 0;
    }
  }

  explicit WirelessController_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lx = 0.0f;
      this->ly = 0.0f;
      this->rx = 0.0f;
      this->ry = 0.0f;
      this->keys = 0;
    }
  }

  // field types and members
  using _lx_type =
    float;
  _lx_type lx;
  using _ly_type =
    float;
  _ly_type ly;
  using _rx_type =
    float;
  _rx_type rx;
  using _ry_type =
    float;
  _ry_type ry;
  using _keys_type =
    uint16_t;
  _keys_type keys;

  // setters for named parameter idiom
  Type & set__lx(
    const float & _arg)
  {
    this->lx = _arg;
    return *this;
  }
  Type & set__ly(
    const float & _arg)
  {
    this->ly = _arg;
    return *this;
  }
  Type & set__rx(
    const float & _arg)
  {
    this->rx = _arg;
    return *this;
  }
  Type & set__ry(
    const float & _arg)
  {
    this->ry = _arg;
    return *this;
  }
  Type & set__keys(
    const uint16_t & _arg)
  {
    this->keys = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::WirelessController_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::WirelessController_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::WirelessController_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::WirelessController_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::WirelessController_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::WirelessController_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::WirelessController_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::WirelessController_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::WirelessController_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::WirelessController_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__WirelessController
    std::shared_ptr<unitree_go::msg::WirelessController_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__WirelessController
    std::shared_ptr<unitree_go::msg::WirelessController_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WirelessController_ & other) const
  {
    if (this->lx != other.lx) {
      return false;
    }
    if (this->ly != other.ly) {
      return false;
    }
    if (this->rx != other.rx) {
      return false;
    }
    if (this->ry != other.ry) {
      return false;
    }
    if (this->keys != other.keys) {
      return false;
    }
    return true;
  }
  bool operator!=(const WirelessController_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WirelessController_

// alias to use template instance with default allocator
using WirelessController =
  unitree_go::msg::WirelessController_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__WIRELESS_CONTROLLER__STRUCT_HPP_
