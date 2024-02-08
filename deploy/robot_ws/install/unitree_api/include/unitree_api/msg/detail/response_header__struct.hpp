// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/ResponseHeader.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__STRUCT_HPP_

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
// Member 'status'
#include "unitree_api/msg/detail/response_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__ResponseHeader __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__ResponseHeader __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ResponseHeader_
{
  using Type = ResponseHeader_<ContainerAllocator>;

  explicit ResponseHeader_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : identity(_init),
    status(_init)
  {
    (void)_init;
  }

  explicit ResponseHeader_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : identity(_alloc, _init),
    status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _identity_type =
    unitree_api::msg::RequestIdentity_<ContainerAllocator>;
  _identity_type identity;
  using _status_type =
    unitree_api::msg::ResponseStatus_<ContainerAllocator>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__identity(
    const unitree_api::msg::RequestIdentity_<ContainerAllocator> & _arg)
  {
    this->identity = _arg;
    return *this;
  }
  Type & set__status(
    const unitree_api::msg::ResponseStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::ResponseHeader_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::ResponseHeader_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::ResponseHeader_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::ResponseHeader_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__ResponseHeader
    std::shared_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__ResponseHeader
    std::shared_ptr<unitree_api::msg::ResponseHeader_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResponseHeader_ & other) const
  {
    if (this->identity != other.identity) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResponseHeader_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResponseHeader_

// alias to use template instance with default allocator
using ResponseHeader =
  unitree_api::msg::ResponseHeader_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_HEADER__STRUCT_HPP_
