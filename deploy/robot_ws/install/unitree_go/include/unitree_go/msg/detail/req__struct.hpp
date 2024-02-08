// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/Req.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__REQ__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__REQ__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__Req __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__Req __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Req_
{
  using Type = Req_<ContainerAllocator>;

  explicit Req_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
      this->body = "";
    }
  }

  explicit Req_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uuid(_alloc),
    body(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
      this->body = "";
    }
  }

  // field types and members
  using _uuid_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _uuid_type uuid;
  using _body_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _body_type body;

  // setters for named parameter idiom
  Type & set__uuid(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }
  Type & set__body(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->body = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::Req_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::Req_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::Req_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::Req_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::Req_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::Req_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::Req_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::Req_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::Req_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::Req_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__Req
    std::shared_ptr<unitree_go::msg::Req_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__Req
    std::shared_ptr<unitree_go::msg::Req_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Req_ & other) const
  {
    if (this->uuid != other.uuid) {
      return false;
    }
    if (this->body != other.body) {
      return false;
    }
    return true;
  }
  bool operator!=(const Req_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Req_

// alias to use template instance with default allocator
using Req =
  unitree_go::msg::Req_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__REQ__STRUCT_HPP_
