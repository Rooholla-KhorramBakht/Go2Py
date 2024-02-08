// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/PathPoint.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__PATH_POINT__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__PATH_POINT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__PathPoint __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__PathPoint __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PathPoint_
{
  using Type = PathPoint_<ContainerAllocator>;

  explicit PathPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t_from_start = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->yaw = 0.0f;
      this->vx = 0.0f;
      this->vy = 0.0f;
      this->vyaw = 0.0f;
    }
  }

  explicit PathPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t_from_start = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->yaw = 0.0f;
      this->vx = 0.0f;
      this->vy = 0.0f;
      this->vyaw = 0.0f;
    }
  }

  // field types and members
  using _t_from_start_type =
    float;
  _t_from_start_type t_from_start;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _vx_type =
    float;
  _vx_type vx;
  using _vy_type =
    float;
  _vy_type vy;
  using _vyaw_type =
    float;
  _vyaw_type vyaw;

  // setters for named parameter idiom
  Type & set__t_from_start(
    const float & _arg)
  {
    this->t_from_start = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__vx(
    const float & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const float & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__vyaw(
    const float & _arg)
  {
    this->vyaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::PathPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::PathPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::PathPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::PathPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::PathPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::PathPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::PathPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::PathPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::PathPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::PathPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__PathPoint
    std::shared_ptr<unitree_go::msg::PathPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__PathPoint
    std::shared_ptr<unitree_go::msg::PathPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathPoint_ & other) const
  {
    if (this->t_from_start != other.t_from_start) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->vyaw != other.vyaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathPoint_

// alias to use template instance with default allocator
using PathPoint =
  unitree_go::msg::PathPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__PATH_POINT__STRUCT_HPP_
