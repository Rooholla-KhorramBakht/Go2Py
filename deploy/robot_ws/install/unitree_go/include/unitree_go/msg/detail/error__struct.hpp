// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__ERROR__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__ERROR__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__Error __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__Error __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Error_
{
  using Type = Error_<ContainerAllocator>;

  explicit Error_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->source = 0ul;
      this->state = 0ul;
    }
  }

  explicit Error_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->source = 0ul;
      this->state = 0ul;
    }
  }

  // field types and members
  using _source_type =
    uint32_t;
  _source_type source;
  using _state_type =
    uint32_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__source(
    const uint32_t & _arg)
  {
    this->source = _arg;
    return *this;
  }
  Type & set__state(
    const uint32_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::Error_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::Error_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::Error_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::Error_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::Error_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::Error_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::Error_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::Error_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::Error_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::Error_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__Error
    std::shared_ptr<unitree_go::msg::Error_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__Error
    std::shared_ptr<unitree_go::msg::Error_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Error_ & other) const
  {
    if (this->source != other.source) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Error_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Error_

// alias to use template instance with default allocator
using Error =
  unitree_go::msg::Error_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__ERROR__STRUCT_HPP_
