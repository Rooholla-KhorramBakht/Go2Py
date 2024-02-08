// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__BMS_CMD__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__BMS_CMD__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__BmsCmd __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__BmsCmd __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BmsCmd_
{
  using Type = BmsCmd_<ContainerAllocator>;

  explicit BmsCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->off = 0;
      std::fill<typename std::array<uint8_t, 3>::iterator, uint8_t>(this->reserve.begin(), this->reserve.end(), 0);
    }
  }

  explicit BmsCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reserve(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->off = 0;
      std::fill<typename std::array<uint8_t, 3>::iterator, uint8_t>(this->reserve.begin(), this->reserve.end(), 0);
    }
  }

  // field types and members
  using _off_type =
    uint8_t;
  _off_type off;
  using _reserve_type =
    std::array<uint8_t, 3>;
  _reserve_type reserve;

  // setters for named parameter idiom
  Type & set__off(
    const uint8_t & _arg)
  {
    this->off = _arg;
    return *this;
  }
  Type & set__reserve(
    const std::array<uint8_t, 3> & _arg)
  {
    this->reserve = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::BmsCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::BmsCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::BmsCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::BmsCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__BmsCmd
    std::shared_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__BmsCmd
    std::shared_ptr<unitree_go::msg::BmsCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BmsCmd_ & other) const
  {
    if (this->off != other.off) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    return true;
  }
  bool operator!=(const BmsCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BmsCmd_

// alias to use template instance with default allocator
using BmsCmd =
  unitree_go::msg::BmsCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__BMS_CMD__STRUCT_HPP_
