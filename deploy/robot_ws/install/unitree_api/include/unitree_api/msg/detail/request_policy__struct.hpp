// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/RequestPolicy.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_POLICY__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_POLICY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__RequestPolicy __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__RequestPolicy __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RequestPolicy_
{
  using Type = RequestPolicy_<ContainerAllocator>;

  explicit RequestPolicy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->priority = 0l;
      this->noreply = false;
    }
  }

  explicit RequestPolicy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->priority = 0l;
      this->noreply = false;
    }
  }

  // field types and members
  using _priority_type =
    int32_t;
  _priority_type priority;
  using _noreply_type =
    bool;
  _noreply_type noreply;

  // setters for named parameter idiom
  Type & set__priority(
    const int32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__noreply(
    const bool & _arg)
  {
    this->noreply = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::RequestPolicy_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::RequestPolicy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestPolicy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestPolicy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__RequestPolicy
    std::shared_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__RequestPolicy
    std::shared_ptr<unitree_api::msg::RequestPolicy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestPolicy_ & other) const
  {
    if (this->priority != other.priority) {
      return false;
    }
    if (this->noreply != other.noreply) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestPolicy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestPolicy_

// alias to use template instance with default allocator
using RequestPolicy =
  unitree_api::msg::RequestPolicy_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_POLICY__STRUCT_HPP_
