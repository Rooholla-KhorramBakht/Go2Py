// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/ResponseStatus.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__RESPONSE_STATUS__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__RESPONSE_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__ResponseStatus __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__ResponseStatus __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ResponseStatus_
{
  using Type = ResponseStatus_<ContainerAllocator>;

  explicit ResponseStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->code = 0l;
    }
  }

  explicit ResponseStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->code = 0l;
    }
  }

  // field types and members
  using _code_type =
    int32_t;
  _code_type code;

  // setters for named parameter idiom
  Type & set__code(
    const int32_t & _arg)
  {
    this->code = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::ResponseStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::ResponseStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::ResponseStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::ResponseStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__ResponseStatus
    std::shared_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__ResponseStatus
    std::shared_ptr<unitree_api::msg::ResponseStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResponseStatus_ & other) const
  {
    if (this->code != other.code) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResponseStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResponseStatus_

// alias to use template instance with default allocator
using ResponseStatus =
  unitree_api::msg::ResponseStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__RESPONSE_STATUS__STRUCT_HPP_
