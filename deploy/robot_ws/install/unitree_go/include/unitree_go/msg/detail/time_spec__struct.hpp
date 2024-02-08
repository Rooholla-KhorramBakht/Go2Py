// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/TimeSpec.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__TIME_SPEC__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__TIME_SPEC__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__TimeSpec __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__TimeSpec __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TimeSpec_
{
  using Type = TimeSpec_<ContainerAllocator>;

  explicit TimeSpec_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sec = 0l;
      this->nanosec = 0ul;
    }
  }

  explicit TimeSpec_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sec = 0l;
      this->nanosec = 0ul;
    }
  }

  // field types and members
  using _sec_type =
    int32_t;
  _sec_type sec;
  using _nanosec_type =
    uint32_t;
  _nanosec_type nanosec;

  // setters for named parameter idiom
  Type & set__sec(
    const int32_t & _arg)
  {
    this->sec = _arg;
    return *this;
  }
  Type & set__nanosec(
    const uint32_t & _arg)
  {
    this->nanosec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::TimeSpec_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::TimeSpec_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::TimeSpec_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::TimeSpec_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__TimeSpec
    std::shared_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__TimeSpec
    std::shared_ptr<unitree_go::msg::TimeSpec_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimeSpec_ & other) const
  {
    if (this->sec != other.sec) {
      return false;
    }
    if (this->nanosec != other.nanosec) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimeSpec_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimeSpec_

// alias to use template instance with default allocator
using TimeSpec =
  unitree_go::msg::TimeSpec_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__TIME_SPEC__STRUCT_HPP_
