// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_HEADER__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_HEADER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'identity'
#include "unitree_api/msg/detail/request_identity__struct.hpp"
// Member 'lease'
#include "unitree_api/msg/detail/request_lease__struct.hpp"
// Member 'policy'
#include "unitree_api/msg/detail/request_policy__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__RequestHeader __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__RequestHeader __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RequestHeader_
{
  using Type = RequestHeader_<ContainerAllocator>;

  explicit RequestHeader_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : identity(_init),
    lease(_init),
    policy(_init)
  {
    (void)_init;
  }

  explicit RequestHeader_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : identity(_alloc, _init),
    lease(_alloc, _init),
    policy(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _identity_type =
    unitree_api::msg::RequestIdentity_<ContainerAllocator>;
  _identity_type identity;
  using _lease_type =
    unitree_api::msg::RequestLease_<ContainerAllocator>;
  _lease_type lease;
  using _policy_type =
    unitree_api::msg::RequestPolicy_<ContainerAllocator>;
  _policy_type policy;

  // setters for named parameter idiom
  Type & set__identity(
    const unitree_api::msg::RequestIdentity_<ContainerAllocator> & _arg)
  {
    this->identity = _arg;
    return *this;
  }
  Type & set__lease(
    const unitree_api::msg::RequestLease_<ContainerAllocator> & _arg)
  {
    this->lease = _arg;
    return *this;
  }
  Type & set__policy(
    const unitree_api::msg::RequestPolicy_<ContainerAllocator> & _arg)
  {
    this->policy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::RequestHeader_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::RequestHeader_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestHeader_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestHeader_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__RequestHeader
    std::shared_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__RequestHeader
    std::shared_ptr<unitree_api::msg::RequestHeader_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestHeader_ & other) const
  {
    if (this->identity != other.identity) {
      return false;
    }
    if (this->lease != other.lease) {
      return false;
    }
    if (this->policy != other.policy) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestHeader_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestHeader_

// alias to use template instance with default allocator
using RequestHeader =
  unitree_api::msg::RequestHeader_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_HEADER__STRUCT_HPP_
