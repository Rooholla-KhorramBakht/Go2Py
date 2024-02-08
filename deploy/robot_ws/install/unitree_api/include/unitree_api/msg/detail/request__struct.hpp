// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/Request.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "unitree_api/msg/detail/request_header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__Request __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__Request __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Request_
{
  using Type = Request_<ContainerAllocator>;

  explicit Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->parameter = "";
    }
  }

  explicit Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    parameter(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->parameter = "";
    }
  }

  // field types and members
  using _header_type =
    unitree_api::msg::RequestHeader_<ContainerAllocator>;
  _header_type header;
  using _parameter_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _parameter_type parameter;
  using _binary_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _binary_type binary;

  // setters for named parameter idiom
  Type & set__header(
    const unitree_api::msg::RequestHeader_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__parameter(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->parameter = _arg;
    return *this;
  }
  Type & set__binary(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->binary = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__Request
    std::shared_ptr<unitree_api::msg::Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__Request
    std::shared_ptr<unitree_api::msg::Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Request_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->parameter != other.parameter) {
      return false;
    }
    if (this->binary != other.binary) {
      return false;
    }
    return true;
  }
  bool operator!=(const Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Request_

// alias to use template instance with default allocator
using Request =
  unitree_api::msg::Request_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST__STRUCT_HPP_
