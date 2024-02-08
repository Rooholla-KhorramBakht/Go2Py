// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_api:msg/RequestIdentity.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__STRUCT_HPP_
#define UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_api__msg__RequestIdentity __attribute__((deprecated))
#else
# define DEPRECATED__unitree_api__msg__RequestIdentity __declspec(deprecated)
#endif

namespace unitree_api
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RequestIdentity_
{
  using Type = RequestIdentity_<ContainerAllocator>;

  explicit RequestIdentity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ll;
      this->api_id = 0ll;
    }
  }

  explicit RequestIdentity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ll;
      this->api_id = 0ll;
    }
  }

  // field types and members
  using _id_type =
    int64_t;
  _id_type id;
  using _api_id_type =
    int64_t;
  _api_id_type api_id;

  // setters for named parameter idiom
  Type & set__id(
    const int64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__api_id(
    const int64_t & _arg)
  {
    this->api_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_api::msg::RequestIdentity_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_api::msg::RequestIdentity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestIdentity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_api::msg::RequestIdentity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_api__msg__RequestIdentity
    std::shared_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_api__msg__RequestIdentity
    std::shared_ptr<unitree_api::msg::RequestIdentity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestIdentity_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->api_id != other.api_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestIdentity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestIdentity_

// alias to use template instance with default allocator
using RequestIdentity =
  unitree_api::msg::RequestIdentity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_api

#endif  // UNITREE_API__MSG__DETAIL__REQUEST_IDENTITY__STRUCT_HPP_
