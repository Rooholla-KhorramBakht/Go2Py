// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/UwbSwitch.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__UWB_SWITCH__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__UWB_SWITCH__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__UwbSwitch __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__UwbSwitch __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UwbSwitch_
{
  using Type = UwbSwitch_<ContainerAllocator>;

  explicit UwbSwitch_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enabled = 0;
    }
  }

  explicit UwbSwitch_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enabled = 0;
    }
  }

  // field types and members
  using _enabled_type =
    uint8_t;
  _enabled_type enabled;

  // setters for named parameter idiom
  Type & set__enabled(
    const uint8_t & _arg)
  {
    this->enabled = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::UwbSwitch_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::UwbSwitch_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::UwbSwitch_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::UwbSwitch_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__UwbSwitch
    std::shared_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__UwbSwitch
    std::shared_ptr<unitree_go::msg::UwbSwitch_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UwbSwitch_ & other) const
  {
    if (this->enabled != other.enabled) {
      return false;
    }
    return true;
  }
  bool operator!=(const UwbSwitch_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UwbSwitch_

// alias to use template instance with default allocator
using UwbSwitch =
  unitree_go::msg::UwbSwitch_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__UWB_SWITCH__STRUCT_HPP_
